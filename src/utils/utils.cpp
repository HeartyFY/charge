#include "utils.h"


namespace utils 
{

uint64_t time_now_ms() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  auto time_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  return time_now_ms;
}
  

}