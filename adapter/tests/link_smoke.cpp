#include "doa_axis_scheduler.h"

int main() {
  doa_axis::Scheduler scheduler;
  return scheduler.active() ? 1 : 0;
}
