#include <cstdint>

#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

#include "core/uart.h"
#include "core/system.h"
#include "comms.h"


#define BOOTLOADER_SIZE (0x8000U) // 32K
#define MAIN_APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)

// This will jump to the entrypoint of 
static void jump_to_application_main()
{
  /*
  typedef void (*void_fn)(void);
  uint32_t* app_vector_table = reinterpret_cast<uint32_t*>(MAIN_APP_START_ADDR);
  void_fn fcnPtr = reinterpret_cast<void_fn>(app_vector_table[1]);

  // maybe reset stack pointer here?

  // Reset vector table and call application
  SCB_VTOR = BOOTLOADER_SIZE;
  fcnPtr();
  SCB_VTOR = 0x0;
  */

  // Start of application is the vector table
  vector_table_t* main_vector_table = reinterpret_cast<vector_table_t*>(MAIN_APP_START_ADDR);

  // Reset the vector table register to point to the new vector table
  SCB_VTOR = BOOTLOADER_SIZE;
  main_vector_table->reset();
}

int main()
{
  system_setup();
  uart_setup();
  comms_setup();

  comms_packet_t packet = {
    .len = 9,
    .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa },
    .crc = 0
  };
  packet.crc = comms_compute_crc(&packet);

  while(true)
  {
    comms_update();
    //comms_write_packet(&packet);
    system_delay(500);
  }
  
  jump_to_application_main();
  // Never return
  return 0;
}
