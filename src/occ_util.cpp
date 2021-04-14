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
#include <occ_util.hpp>

#include <endian.h>
#include <set>
#include <map>
#include <string>

#include <occ_sensor_t.hpp>
#include <occ.hpp>

std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occid(const std::set<occ_sensor_t>& sensors) {
    std::map<std::string, std::set<occ_sensor_t>> m;
    for (const auto& s : sensors) {
        m[s.name].insert(s);
    }
    return m;
}

std::map<occ_sensor_t, double> get_sensor_values(void* buf, const std::set<occ_sensor_t>& requested_sensors) {
    std::map<occ_sensor_t, double> values_by_sensor;
    auto sensors_by_occid = get_sensors_by_occid(requested_sensors);

	struct occ_sensor_data_header* hb = (struct occ_sensor_data_header *)(uint64_t)buf;
	struct occ_sensor_name* md = (struct occ_sensor_name *)((uint64_t)hb + be32toh(hb->names_offset));

    // iterate over all sensors
	for (int i = 0; i < be16toh(hb->nr_sensors); i++) {
        // check if current sensor is in requested_sensors
        if (sensors_by_occid.find(md[i].name) != sensors_by_occid.end()) {
            for (const auto& sensor : sensors_by_occid[md[i].name]) {
                if (!sensor.acc) {
                    // regular sensor
                    uint32_t scale = be32toh(md[i].scale_factor);
                    uint64_t sample = get_sample(hb, &md[i]);
                    values_by_sensor[sensor] = sample * get_occ_scale_as_fp(scale);
                } else {
                    // accumulator
                    uint64_t energy = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_ACCUMULATOR);
                    uint32_t freq = be32toh(md[i].freq);
                    values_by_sensor[sensor] = energy / get_occ_scale_as_fp(freq);
                }
            }
        }
	}

    if (requested_sensors.size() != values_by_sensor.size()) {
        throw std::runtime_error("could not find all sensors in extraced file (expected: " + std::to_string(requested_sensors.size()) + ", got: " + std::to_string(values_by_sensor.size()) + ")");
    }

    return values_by_sensor;
}
