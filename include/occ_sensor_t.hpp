#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__

#include <string>
#include <map>

#include <scorep/plugin/plugin.hpp>
#include <scorep/SCOREP_MetricTypes.h>


/// contains all information required to locate a sensor and grab its value
struct occ_sensor_t {
    occ_sensor_t(const std::string& name, const bool acc) : name(name), acc(acc) {}
    occ_sensor_t() {}

    /// name as used in occ_inband_sensors (e.g. PWRSYS)
    std::string name;
    /// whether to use the accumulator
    bool acc = false;

    /// metric properties for all supported sensors
    static const std::map<occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor;
};
typedef struct occ_sensor_t occ_sensor_t;


bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs); 

template <typename T, typename Policies>
using occ_sensor_policy = scorep::plugin::policy::object_id<occ_sensor_t, T, Policies>;

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__
