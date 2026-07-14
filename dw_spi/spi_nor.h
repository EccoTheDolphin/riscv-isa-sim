#ifndef RISCV_SPI_NOR_H
#define RISCV_SPI_NOR_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Minimal Winbond W25Q128JV model. Status and command behavior follows
// Sections 7 and 8 of the W25Q128JV Revision F datasheet:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
class spi_nor_t {
 public:
  static constexpr size_t flash_size = 16 * 1024 * 1024;
  static constexpr size_t page_size = 256;
  static constexpr size_t sector_size = 64 * 1024;

  explicit spi_nor_t(std::vector<uint8_t> storage);

  void cs_assert();
  uint8_t transfer(uint8_t tx);
  void cs_deassert();
  bool read(size_t address, size_t len, uint8_t* bytes) const;

 private:
  static constexpr uint8_t erased_value = 0xff;
  static constexpr uint8_t status_wel = 1 << 1;

  enum class command_t : uint8_t {
    none,
    unknown,
    read_id,
    read_status,
    write_enable,
    read,
    page_program,
    sector_erase,
    chip_erase,
  };

  void decode_command(uint8_t opcode);
  bool receive_address_byte(uint8_t value);
  uint8_t status() const;
  void commit_page_program();
  void commit_sector_erase();
  void commit_chip_erase();
  void reset_transaction();

  std::vector<uint8_t> storage;
  std::array<uint8_t, page_size> program_buffer;
  command_t command;
  uint32_t address;
  size_t address_bytes;
  size_t response_index;
  size_t next_program_offset;
  bool cs_active;
  bool write_enable_latch;
  bool program_data_received;
};

#endif
