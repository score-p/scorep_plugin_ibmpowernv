// Copyright (C) 2021 Technische Universität Dresden, Germany
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// (1) Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in
// the documentation and/or other materials provided with the
// distribution.
//
// (3)The name of the author may not be used to
// endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__

#include <map>
#include <set>
#include <string>

#include <occ_sensor_t.hpp>

/**
 * struct to hold all information to reconstruct a sensor (different data types etc.)
 */
struct all_sample_data {
    double fp64 = 0;
    uint64_t int_unsigned = 0;
    int64_t int_signed = 0;

    uint64_t acc_raw = 0;
    uint64_t update_tag = 0;
    double acc_freq = 0;

    /// automatically fill values
    void fill(uint64_t u);
    void fill(double d);
};
typedef struct all_sample_data all_sample_data;

/**
 * store data as provided by OCC for one sensor
 */
struct sensor_data_t {
    /// timestamp as reported by OCC (512 MHz-based)
    uint64_t timestamp;

    /// sample (scale already applied)
    double sample;

    /// accumulator as reported by OCC
    uint64_t accumulator;

    /// number of samples stored in accumulator
    uint64_t update_tag;
};
using sensor_data_t = struct sensor_data_t;

/**
 * reorganize given sensors into map with OCC identifier (string like "PWRSYS") -> sensor(s).
 * helper used for easier lookup when iterating all available sensors.
 * @param sensors sensor to be reorganized
 * @return map: OCC identifier -> legacy sensor object
 */
std::map<std::string, std::set<legacy_occ_sensor_t>> get_sensors_by_occid(const std::set<legacy_occ_sensor_t>& sensors);

/**
 * reorganize given sensors into map with OCC name (string like "PWRSYS") -> sensor(s).
 * helper used for easier lookup when iterating all available sensors.
 * @param sensors sensor to be reorganized
 * @return map: OCC identifier -> sensor object
 */
std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occ_name(const std::set<occ_sensor_t>& sensors);

/**
 * extract a set of sensor values from given occ inband sensors file.
 * @param buf pointer to content of occ inband sensors file
 * @param requested_sensors sensor to be extracted
 * @param socket_count number of sockets to search
 * @return map: sensors type -> value from buf
 */
std::map<legacy_occ_sensor_t, all_sample_data> get_sensor_values(void* buf,
                                                                 const std::set<legacy_occ_sensor_t>& requested_sensors,
                                                                 const int socket_count);

/**
 * extract sensor readouts from given occ inband sensors file.
 * @param buf pointer to content of occ inband sensors file
 * @param requested_sensors sensor to be extracted
 * @param socket_count number of sockets to search
 * @return map: sensors type -> read sensor data
 */
std::map<occ_sensor_t, sensor_data_t> get_sensor_data(void* buf,
                                                      const std::set<occ_sensor_t>& requested_sensors,
                                                      const int socket_count);

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_UTIL_HPP_INCLUDED__
