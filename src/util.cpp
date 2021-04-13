#include <util.hpp>

#include <chrono>
#include <string>
#include <stdexcept>
#include <regex>

std::chrono::nanoseconds get_ns_from_str(const std::string& interval_str)
{
    // note: regex compilation crashes on our test machines, presumably due to linker issues
    std::regex r("([0-9]+)([mun]?s)");
    std::smatch s;

    if (std::regex_match(interval_str, s, r)) {
        std::string time = s[1];
        std::string unit = s[2];

        switch (unit[0]) {
        case 's':
            return std::chrono::seconds(std::stol(time));
        case 'm':
            return std::chrono::milliseconds(std::stol(time));
        case 'u':
            return std::chrono::microseconds(std::stol(time));
        case 'n':
            return std::chrono::nanoseconds(std::stol(time));
        }
    }

    throw std::runtime_error("could not parse time: " + interval_str);
}
