#include <stdio.h>
#include <stdlib.h>

extern "C"
{
#include <libacpi.h>
}
int main() {
  global_t * globals = (global_t *)malloc(sizeof(global_t));
  init_acpi_batt(globals);
  printf("#%d\n",globals->batt_count);
  read_acpi_batt(0);
  battery_t batt = batteries[0];
  printf("%d %d %d\n", batt.percentage, batt.charge_time, batt.remaining_time);  
  return 0;
}
