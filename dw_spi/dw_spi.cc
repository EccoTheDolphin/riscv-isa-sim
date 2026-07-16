#include "dw_spi.h"

#include <charconv>
#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

// Public DesignWare SPI register definitions:
// https://github.com/torvalds/linux/blob/master/drivers/spi/spi-dw.h
constexpr reg_t register_ctrlr0 = 0x00;
constexpr reg_t register_ssienr = 0x08;
constexpr reg_t register_ser = 0x10;
constexpr reg_t register_baudr = 0x14;
constexpr reg_t register_sr = 0x28;
constexpr reg_t register_imr = 0x2c;
constexpr reg_t register_dr = 0x60;

constexpr reg_t flash_window_offset = 0x1000;
constexpr reg_t minimum_device_size =
  flash_window_offset + spi_nor_t::flash_size;
constexpr size_t tx_fifo_capacity = 16;

constexpr uint32_t supported_ctrlr0 = 0x7;
constexpr uint32_t supported_ser = 0x1;

constexpr uint8_t status_busy = 1 << 0;
constexpr uint8_t status_tfnf = 1 << 1;
constexpr uint8_t status_tfe = 1 << 2;
constexpr uint8_t status_rfne = 1 << 3;

struct dw_spi_config_t {
  reg_t mmio_base;
  reg_t mmio_size;
  std::string image_path;
  spi_nor_t::jedec_id_t jedec_id;
};

reg_t parse_reg(const std::string& value, const char* name)
{
  std::string_view input = value;
  int base = 10;
  if (input.starts_with("0x") || input.starts_with("0X")) {
    input.remove_prefix(2);
    base = 16;
  }

  reg_t result = 0;
  const auto [end, error] =
    std::from_chars(input.data(), input.data() + input.size(), result, base);
  if (error == std::errc::result_out_of_range)
    throw std::out_of_range(std::string(name) + " does not fit in reg_t");
  if (error != std::errc{} || end != input.data() + input.size())
    throw std::invalid_argument(std::string(name) + " must be an unsigned integer");

  return result;
}

uint32_t read_little_endian_u32(const uint8_t* bytes)
{
  return uint32_t(bytes[0]) |
         uint32_t(bytes[1]) << 8 |
         uint32_t(bytes[2]) << 16 |
         uint32_t(bytes[3]) << 24;
}

spi_nor_t::jedec_id_t parse_jedec_id(const std::string& value)
{
  const reg_t id = parse_reg(value, "jedec");
  if (id > 0xffffff)
    throw std::out_of_range("jedec must fit in 24 bits");

  return {
    static_cast<uint8_t>(id >> 16),
    static_cast<uint8_t>(id >> 8),
    static_cast<uint8_t>(id),
  };
}

dw_spi_config_t parse_config(const std::vector<std::string>& args)
{
  if (args.size() < 2)
    throw std::invalid_argument(
      "expected <base>,<mmio-size>[,img=<path>][,jedec=<id>]");

  dw_spi_config_t config = {
    .mmio_base = parse_reg(args[0], "base"),
    .mmio_size = parse_reg(args[1], "mmio-size"),
    .image_path = {},
    .jedec_id = spi_nor_t::default_jedec_id,
  };

  if (config.mmio_base == 0)
    throw std::invalid_argument("base must be nonzero");
  if (config.mmio_size < minimum_device_size)
    throw std::invalid_argument("mmio-size must be at least 0x1001000");
  if (config.mmio_base >
      std::numeric_limits<reg_t>::max() - (config.mmio_size - 1))
    throw std::out_of_range("device address range overflows reg_t");

  bool image_seen = false;
  bool jedec_seen = false;
  for (size_t i = 2; i < args.size(); ++i) {
    const size_t separator = args[i].find('=');
    if (separator == std::string::npos)
      throw std::invalid_argument("optional arguments must use key=value");

    const std::string_view key(args[i].data(), separator);
    const std::string value = args[i].substr(separator + 1);
    if (key == "img") {
      if (image_seen)
        throw std::invalid_argument("img specified more than once");
      if (value.empty())
        throw std::invalid_argument("img path must be nonempty");
      config.image_path = value;
      image_seen = true;
      continue;
    }

    if (key == "jedec") {
      if (jedec_seen)
        throw std::invalid_argument("jedec specified more than once");
      if (value.empty())
        throw std::invalid_argument("jedec must be nonempty");
      config.jedec_id = parse_jedec_id(value);
      jedec_seen = true;
      continue;
    }

    throw std::invalid_argument("unknown option '" + std::string(key) + "'");
  }

  return config;
}

std::vector<uint8_t>
load_initial_image(const std::string& image_path, size_t image_size)
{
  std::vector<uint8_t> image(image_size, 0xff);
  if (image_path.empty())
    return image;

  std::ifstream input(image_path, std::ios::binary | std::ios::ate);
  if (!input)
    throw std::runtime_error("cannot open initial image '" + image_path + "'");

  const std::streamoff file_size = input.tellg();
  if (file_size < 0)
    throw std::runtime_error("cannot determine initial image size");
  if (static_cast<size_t>(file_size) > image_size)
    throw std::invalid_argument("initial image exceeds storage size");

  input.seekg(0, std::ios::beg);
  if (!input)
    throw std::runtime_error("cannot seek initial image");

  if (file_size != 0) {
    input.read(reinterpret_cast<char*>(image.data()), file_size);
    if (!input)
      throw std::runtime_error("cannot read complete initial image");
  }

  return image;
}

dw_spi_t* dw_spi_parse_from_fdt(const void*, const sim_t*, reg_t* base,
                                const std::vector<std::string>& args)
{
  try {
    const auto config = parse_config(args);
    auto initial_image =
      load_initial_image(config.image_path, spi_nor_t::flash_size);
    *base = config.mmio_base;
    return new dw_spi_t(config.mmio_size, std::move(initial_image),
                        config.jedec_id);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "dw_spi: invalid device configuration: %s\n",
                 error.what());
    return nullptr;
  }
}

std::string dw_spi_generate_dts(const sim_t*,
                                const std::vector<std::string>&)
{
  return {};
}

} // namespace

dw_spi_t::dw_spi_t(reg_t device_size, std::vector<uint8_t> initial_image,
                   spi_nor_t::jedec_id_t jedec_id)
  : device_size(device_size), flash(std::move(initial_image), jedec_id),
    ctrlr0(0), ser(0), baudr(0), imr(0), idle_poll_seen(false), enabled(false),
    transaction_active(false)
{
}

bool dw_spi_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  if (len == 0 || addr > device_size || len > device_size - addr)
    return false;

  if (addr >= flash_window_offset) {
    const reg_t flash_address = addr - flash_window_offset;
    if (flash_address > spi_nor_t::flash_size ||
        len > spi_nor_t::flash_size - flash_address)
      return false;

    return flash.read(static_cast<size_t>(flash_address), len, bytes);
  }

  return load_controller(addr, len, bytes);
}

bool dw_spi_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if (len == 0 || addr > device_size || len > device_size - addr)
    return false;

  if (addr >= flash_window_offset)
    return false;

  return store_controller(addr, len, bytes);
}

reg_t dw_spi_t::size()
{
  return device_size;
}

bool dw_spi_t::load_controller(reg_t addr, size_t len, uint8_t* bytes)
{
  if (len != 1)
    return false;

  switch (addr) {
    case register_sr:
      progress_transfer();
      bytes[0] = status();
      return true;
    case register_dr:
      if (rx_fifo.empty())
        return false;
      bytes[0] = rx_fifo.front();
      rx_fifo.pop_front();
      idle_poll_seen = false;
      return true;
    default:
      return false;
  }
}

bool dw_spi_t::store_controller(reg_t addr, size_t len,
                                const uint8_t* bytes)
{
  if (addr == register_dr) {
    if (len != 1 || !enabled || ser != supported_ser ||
        tx_fifo.size() >= tx_fifo_capacity)
      return false;

    if (!transaction_active) {
      flash.cs_assert();
      transaction_active = true;
    }

    tx_fifo.push_back(bytes[0]);
    idle_poll_seen = false;
    return true;
  }

  if (len != sizeof(uint32_t))
    return false;

  const uint32_t value = read_little_endian_u32(bytes);
  if (addr == register_ssienr) {
    if (value > 1)
      return false;
    if (value == 0)
      disable_controller();
    else
      enabled = true;
    return true;
  }

  if (enabled)
    return false;

  switch (addr) {
    case register_ctrlr0:
      if (value != supported_ctrlr0)
        return false;
      ctrlr0 = value;
      return true;
    case register_ser:
      if (value != supported_ser)
        return false;
      ser = value;
      return true;
    case register_baudr:
      if (value == 0 || (value & 1) != 0)
        return false;
      baudr = value;
      return true;
    case register_imr:
      if (value != 0)
        return false;
      imr = value;
      return true;
    default:
      return false;
  }
}

uint8_t dw_spi_t::status() const
{
  uint8_t result = 0;
  if (transaction_active)
    result |= status_busy;
  if (tx_fifo.size() < tx_fifo_capacity)
    result |= status_tfnf;
  if (tx_fifo.empty())
    result |= status_tfe;
  if (!rx_fifo.empty())
    result |= status_rfne;
  return result;
}

void dw_spi_t::progress_transfer()
{
  if (!enabled)
    return;

  if (!tx_fifo.empty()) {
    const uint8_t tx = tx_fifo.front();
    tx_fifo.pop_front();
    rx_fifo.push_back(flash.transfer(tx));
    idle_poll_seen = false;
    return;
  }

  if (!transaction_active)
    return;

  // One empty SR poll may precede another DR access. A second empty poll
  // without intervening DR activity is wait_tx_finish() releasing CS.
  if (idle_poll_seen)
    finish_transaction();
  else
    idle_poll_seen = true;
}

void dw_spi_t::finish_transaction()
{
  if (transaction_active)
    flash.cs_deassert();
  transaction_active = false;
  idle_poll_seen = false;
}

void dw_spi_t::disable_controller()
{
  finish_transaction();
  enabled = false;
  tx_fifo.clear();
  rx_fifo.clear();
}

REGISTER_DEVICE(dw_spi, dw_spi_parse_from_fdt, dw_spi_generate_dts)
