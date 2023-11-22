#include <cstdint>

#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/scb.h>

#define BOOTLOADER_SIZE (0x8000U) // 32K
#define MAIN_APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)

// This will jump to the entrypoint of 
void jump_to_application_main()
{
  uint32_t* reset_vector_entry = reinterpret_cast<uint32_t*>(MAIN_APP_START_ADDR + 4U);
  // address of reset_vector is stored in the reset_vector_entry
  uint32_t* reset_vector = reinterpret_cast<uint32_t*>(*reset_vector_entry);

  void (*fcnPtr)(void) = reinterpret_cast<void (*)(void)>(reset_vector);

  // maybe reset stack pointer here?

  // Reset vector table and call application
  SCB_VTOR = BOOTLOADER_SIZE;
  fcnPtr();
}
int main()
{
  jump_to_application_main();
  // Never return
  return 0;
}
