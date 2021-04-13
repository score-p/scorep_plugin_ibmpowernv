#ifndef _SCOREP_IBMPOWERNV_PLUGIN_UTIL_HPP_DEFINED__
#define _SCOREP_IBMPOWERNV_PLUGIN_UTIL_HPP_DEFINED__

#include <chrono>
#include <string>

/**
 * parse string given via environment vars to chrono::duration
 * @param interval_str string to parse, e.g. "10ms"
 * @return duration corresponding to that string
 * @throws std::runtime_error on parsing error
 */
std::chrono::nanoseconds get_ns_from_str(const std::string& interval_str);

#endif // _SCOREP_IBMPOWERNV_PLUGIN_UTIL_HPP_DEFINED__
