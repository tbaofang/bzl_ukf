
#include "customized_time.h"

#include <string>

namespace common {

double ToSeconds(const Duration duration) 
{
  return std::chrono::duration_cast<std::chrono::duration<double> >(duration).count();
}

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }


int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) 
{
  os << std::to_string(ToUniversal(time));
  return os;
}

}  // namespace common

