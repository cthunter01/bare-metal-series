#include <core/system.h>

int main(void)
{
begin_main:
  system_setup();

  while (1)
  {
  }

// If for some reason we broke out of the while loop,
// go back to beginning of main().
goto begin_main;
  // Never return
  return 0;
}
