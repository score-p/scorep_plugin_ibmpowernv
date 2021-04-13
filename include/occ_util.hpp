#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__

#include <set>
#include <map>
#include <string>

#include <occ_sensor_t.hpp>

/// create map with OCC identifier (string like "PWRSYS") -> sensor(s)
std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occid(const std::set<occ_sensor_t>& sensors);

/// extract sample from given buffer
uint64_t get_sample(struct occ_sensor_data_header* header_buffer, struct occ_sensor_name* md);

/// extract values from occ inband sensors file for given sensors
std::map<occ_sensor_t, double> get_sensor_values(void* buf, const std::set<occ_sensor_t>& requested_sensors);

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
