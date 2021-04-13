#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__

#include <set>
#include <map>
#include <string>

#include <occ_sensor_t.hpp>

/**
 * reorganize given sensors into map with OCC identifier (string like "PWRSYS") -> sensor(s).
 * helper used for easier lookup when iterating all available sensors.
 * @param sensors sensor to be reorganized
 * @return map: OCC identifiert -> sensor object
 */
std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occid(const std::set<occ_sensor_t>& sensors);

/**
 * extract a set of sensor values from given occ inband sensors file.
 * @param buf pointer to content of occ inband sensors file
 * @param requested_sensors sensor to be extracted
 * @return map: sensors type -> value from buf
 */
std::map<occ_sensor_t, double> get_sensor_values(void* buf, const std::set<occ_sensor_t>& requested_sensors);

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
