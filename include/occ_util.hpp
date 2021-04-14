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
