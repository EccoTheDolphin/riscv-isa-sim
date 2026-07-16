#ifndef RISCV_DW_SPI_H
#define RISCV_DW_SPI_H

#include "abstract_device.h"
#include "spi_nor.h"

#include <cstdint>
#include <deque>
#include <vector>

class dw_spi_t : public abstract_device_t {
 public:
  dw_spi_t(reg_t device_size, std::vector<uint8_t> initial_image,
           spi_nor_t::jedec_id_t jedec_id);

  bool load(reg_t addr, size_t len, uint8_t* bytes) override;
  bool store(reg_t addr, size_t len, const uint8_t* bytes) override;
  reg_t size() override;

 private:
  bool load_controller(reg_t addr, size_t len, uint8_t* bytes);
  bool store_controller(reg_t addr, size_t len, const uint8_t* bytes);
  uint8_t status() const;
  void progress_transfer();
  void finish_transaction();
  void disable_controller();

  reg_t device_size;
  spi_nor_t flash;
  std::deque<uint8_t> tx_fifo;
  std::deque<uint8_t> rx_fifo;
  uint32_t ctrlr0;
  uint32_t ser;
  uint32_t baudr;
  uint32_t imr;
  bool idle_poll_seen;
  bool enabled;
  bool transaction_active;
};

#endif
