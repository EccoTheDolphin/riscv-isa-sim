#include "spi_nor.h"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace {

using sfdp_data_t = std::array<uint8_t, 256>;

// W25Q128JV Rev. F, Sections 8.1.2 and 8.2:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
constexpr uint8_t command_read_id = 0x9f;
constexpr uint8_t command_read_sfdp = 0x5a;
constexpr uint8_t command_read_status = 0x05;
constexpr uint8_t command_write_enable = 0x06;
constexpr uint8_t command_read = 0x03;
constexpr uint8_t command_page_program = 0x02;
constexpr uint8_t command_sector_erase = 0xd8;
constexpr uint8_t command_chip_erase = 0xc7;

// W25Q128JV Rev. F, Section 8.2.28 defines a 256-byte SFDP register read
// with command 5Ah, a 24-bit address, and eight dummy clocks:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
// These register values come from a W25Q128JV hardware dump published with
// the Linux SPI NOR driver review:
// https://lists.infradead.org/pipermail/linux-mtd/2023-July/099629.html
constexpr std::array<uint8_t, 16> sfdp_header = {
  0x53, 0x46, 0x44, 0x50, 0x05, 0x01, 0x00, 0xff,
  0x00, 0x05, 0x01, 0x10, 0x80, 0x00, 0x00, 0xff,
};

constexpr std::array<uint8_t, 64> sfdp_basic_flash_parameters = {
  0xe5, 0x20, 0xf9, 0xff, 0xff, 0xff, 0xff, 0x07,
  0x44, 0xeb, 0x08, 0x6b, 0x08, 0x3b, 0x42, 0xbb,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0xff, 0xff, 0x40, 0xeb, 0x0c, 0x20, 0x0f, 0x52,
  0x10, 0xd8, 0x00, 0x00, 0x36, 0x02, 0xa6, 0x00,
  0x82, 0xea, 0x14, 0xc9, 0xe9, 0x63, 0x76, 0x33,
  0x7a, 0x75, 0x7a, 0x75, 0xf7, 0xa2, 0xd5, 0x5c,
  0x19, 0xf7, 0x4d, 0xff, 0xe9, 0x30, 0xf8, 0x80,
};

constexpr size_t sfdp_basic_flash_parameters_offset = 0x80;

constexpr sfdp_data_t sfdp_data = []() -> sfdp_data_t {
  sfdp_data_t data{};
  data.fill(0xff);
  for (size_t i = 0; i < sfdp_header.size(); ++i)
    data[i] = sfdp_header[i];
  for (size_t i = 0; i < sfdp_basic_flash_parameters.size(); ++i)
    data[sfdp_basic_flash_parameters_offset + i] =
      sfdp_basic_flash_parameters[i];
  return data;
}();

static_assert((spi_nor_t::page_size & (spi_nor_t::page_size - 1)) == 0);
static_assert((spi_nor_t::sector_size & (spi_nor_t::sector_size - 1)) == 0);
static_assert(spi_nor_t::flash_size % spi_nor_t::page_size == 0);
static_assert(spi_nor_t::flash_size % spi_nor_t::sector_size == 0);

} // namespace

spi_nor_t::spi_nor_t(std::vector<uint8_t> storage, jedec_id_t jedec_id)
  : storage(std::move(storage)), jedec_id(jedec_id), program_buffer{},
    command(command_t::none), address(0), address_bytes(0), response_index(0),
    next_program_offset(0), cs_active(false), write_enable_latch(false),
    program_data_received(false)
{
  if (this->storage.size() != flash_size)
    throw std::invalid_argument("SPI NOR initial image must be exactly 16 MiB");
  reset_transaction();
}

void spi_nor_t::cs_assert()
{
  reset_transaction();
  cs_active = true;
}

uint8_t spi_nor_t::transfer(uint8_t tx)
{
  if (!cs_active)
    return erased_value;

  if (command == command_t::none) {
    decode_command(tx);
    return erased_value;
  }

  switch (command) {
    case command_t::read_id:
      if (response_index < jedec_id.size())
        return jedec_id[response_index++];
      return erased_value;

    case command_t::read_sfdp:
      if (receive_address_byte(tx))
        return erased_value;
      if (response_index++ == 0)
        return erased_value;
      {
        const uint8_t result = address < sfdp_data.size()
                                 ? sfdp_data[address]
                                 : erased_value;
        ++address;
        return result;
      }

    case command_t::read_status:
      return status();

    case command_t::read:
      if (receive_address_byte(tx))
        return erased_value;
      {
        const uint8_t result = storage[address];
        address = (address + 1) % flash_size;
        return result;
      }

    case command_t::page_program:
      if (receive_address_byte(tx)) {
        if (address_bytes == 3)
          next_program_offset = address & (page_size - 1);
      } else {
        program_buffer[next_program_offset] = tx;
        next_program_offset = (next_program_offset + 1) & (page_size - 1);
        program_data_received = true;
      }
      return erased_value;

    case command_t::sector_erase:
      receive_address_byte(tx);
      return erased_value;

    case command_t::write_enable:
    case command_t::chip_erase:
    case command_t::unknown:
    case command_t::none:
      return erased_value;
  }

  return erased_value;
}

void spi_nor_t::cs_deassert()
{
  if (!cs_active)
    return;

  switch (command) {
    case command_t::write_enable:
      write_enable_latch = true;
      break;
    case command_t::page_program:
      commit_page_program();
      break;
    case command_t::sector_erase:
      commit_sector_erase();
      break;
    case command_t::chip_erase:
      commit_chip_erase();
      break;
    case command_t::none:
    case command_t::unknown:
    case command_t::read_id:
    case command_t::read_sfdp:
    case command_t::read_status:
    case command_t::read:
      break;
  }

  cs_active = false;
  reset_transaction();
}

bool spi_nor_t::read(size_t address, size_t len, uint8_t* bytes) const
{
  if (address > storage.size() || len > storage.size() - address)
    return false;

  std::copy_n(storage.begin() + address, len, bytes);
  return true;
}

void spi_nor_t::decode_command(uint8_t opcode)
{
  switch (opcode) {
    case command_read_id:
      command = command_t::read_id;
      break;
    case command_read_sfdp:
      command = command_t::read_sfdp;
      break;
    case command_read_status:
      command = command_t::read_status;
      break;
    case command_write_enable:
      command = command_t::write_enable;
      break;
    case command_read:
      command = command_t::read;
      break;
    case command_page_program:
      command = command_t::page_program;
      break;
    case command_sector_erase:
      command = command_t::sector_erase;
      break;
    case command_chip_erase:
      command = command_t::chip_erase;
      break;
    default:
      command = command_t::unknown;
      break;
  }
}

bool spi_nor_t::receive_address_byte(uint8_t value)
{
  if (address_bytes == 3)
    return false;

  address = (address << 8) | value;
  ++address_bytes;
  if (address_bytes == 3)
    address %= flash_size;
  return true;
}

uint8_t spi_nor_t::status() const
{
  uint8_t result = 0;
  // Program and erase complete synchronously, so WIP is always clear.
  if (write_enable_latch)
    result |= status_wel;
  return result;
}

void spi_nor_t::commit_page_program()
{
  if (!write_enable_latch || address_bytes != 3 || !program_data_received)
    return;

  const size_t page_base = address & ~(page_size - 1);
  for (size_t i = 0; i < program_buffer.size(); ++i)
    storage[page_base + i] &= program_buffer[i];
  write_enable_latch = false;
}

void spi_nor_t::commit_sector_erase()
{
  if (!write_enable_latch || address_bytes != 3)
    return;

  // Section 8.2.17 calls D8h a 64KB Block Erase; OpenOCD calls it a sector.
  const size_t sector_base = address & ~(sector_size - 1);
  std::fill_n(storage.begin() + sector_base, sector_size, erased_value);
  write_enable_latch = false;
}

void spi_nor_t::commit_chip_erase()
{
  if (!write_enable_latch)
    return;

  std::fill(storage.begin(), storage.end(), erased_value);
  write_enable_latch = false;
}

void spi_nor_t::reset_transaction()
{
  command = command_t::none;
  address = 0;
  address_bytes = 0;
  response_index = 0;
  program_buffer.fill(erased_value);
  next_program_offset = 0;
  program_data_received = false;
}
