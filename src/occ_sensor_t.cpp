#include <occ_sensor_t.hpp>

#include <scorep/SCOREP_MetricTypes.h>

#include <string>
#include <map>

const std::map<occ_sensor_t, scorep::plugin::metric_property> occ_sensor_t::metric_properties_by_sensor = {
    // structure:
    // OCC-string identifier, bool whether to use accumulator or not
    // name for metric in trace, description, unit
    {
        {"PWRSYS", false},
        scorep::plugin::metric_property("occ_power_system", "power intake of the entire system", "W")
    },
    {
        {"PWRSYS", true},
        scorep::plugin::metric_property("occ_power_system_acc", "accumulator of consumed energy by entire system", "J")
    },
};

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    if (lhs.name != rhs.name) {
        return lhs.name < rhs.name;
    }

    return lhs.acc < rhs.acc;
}
