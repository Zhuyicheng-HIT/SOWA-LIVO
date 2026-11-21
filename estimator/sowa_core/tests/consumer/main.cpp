#include <sowa/core.h>

int main() {
  sowa::PriorityGate gate;
  gate.reset();
  return gate.active() ? 1 : 0;
}
