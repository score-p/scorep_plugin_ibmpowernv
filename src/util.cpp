#include <util.hpp>

#include <chrono>
#include <string>
#include <stdexcept>
#include <regex>
#include <cassert>

using std::chrono::duration_cast;

std::chrono::nanoseconds get_ns_from_str(const std::string& str)
{
    // copyright notice:
    // this code is taken from metricq-cpp, more precisely metricq-cpp/src/chrono.cpp
    // licensed under the BSD-3-clause license, originally with (c)  ZIH, Technische Universitaet Dresden, Federal Republic of Germany
    // for a full text of the BSD-3-clause license please refer to the license of this code
    
    // Note that regex_match only considers full matches, so no ^$ needed
    static std::regex number_and_unit("\\s*([+-]?\\d*[.,]?\\d+)\\s*([^\\d]*)\\s*");
    std::smatch match;
    if (!std::regex_match(str, match, number_and_unit))
    {
        throw std::invalid_argument("invalid duration string \"" + str +
                                    "\", not of form \"number unit\"");
    }
    assert(match.size() == 3);
    double value = std::stod(match[1]);
    std::string unit = match[2];
    // TODO C++20 use .starts_with
    // Note: we use duration<double> to avoid distinguishing between double and integral types
    // If you specify like 53 days as "4579200000000000 nanosecond" you will get rounding errors
    // Let the duration cast figure out the nifty details, hopefully correctly
    if (unit == "" or unit == "s" or unit == "second" or unit == "seconds")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(value));
    }
    if (unit == "ms" or unit == "millisecond" or unit == "milliseconds")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::milli>(value));
    }
    if (unit == "us" or unit == "microsecond" or unit == "microseconds" or unit == "μs")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::micro>(value));
    }
    if (unit == "ns" or unit == "nanosecond" or unit == "nanoseconds")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::nano>(value));
    }
    if (unit == "min" or unit == "minute" or unit == "minutes")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::ratio<60>>(value));
    }
    if (unit == "h" or unit == "hour" or unit == "hours")
    {
        return duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double, std::ratio<3600>>(value));
    }
 
   throw std::invalid_argument("invalid duration unit \"" + unit + "\"");
}
