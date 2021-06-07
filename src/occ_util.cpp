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
#include <map>
#include <set>
#include <string>

#include <occ.hpp>
#include <occ_sensor_t.hpp>


void all_sample_data::fill(uint64_t u) {
    int_unsigned = u;
    int_signed = u;
    fp64 = u;
}

void all_sample_data::fill(double d) {
    fp64 = d;
    int_signed = (int64_t) (d + 0.5);
    int_unsigned = int_signed;
}

std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occid(const std::set<occ_sensor_t>& sensors)
{
    std::map<std::string, std::set<occ_sensor_t>> m;
    for (const auto& s : sensors) {
        m[s.name].insert(s);
    }
    return m;
}

static std::map<occ_sensor_t, all_sample_data> get_sensor_values_single_socket(void* buf, const std::set<occ_sensor_t>& requested_sensors, const int socket_num)
{
    std::map<occ_sensor_t, all_sample_data> values_by_sensor;
    auto sensors_by_occid = get_sensors_by_occid(requested_sensors);

    // note: apply offset for current socket
    // this looks a little strange, so let me elaborate briefly:
    // cast buf to uint8_t, so any addition by operator+ is in byte
    // then add the required offset and cast to final type
    struct occ_sensor_data_header* hb = (struct occ_sensor_data_header*)((uint8_t*)buf + socket_num * OCC_SENSOR_DATA_BLOCK_SIZE);
    struct occ_sensor_name* md =
        (struct occ_sensor_name*)((uint64_t)hb + be32toh(hb->names_offset));

    // iterate over all sensors
    for (int i = 0; i < be16toh(hb->nr_sensors); i++) {
        // check if current sensor is in requested_sensors
        if (sensors_by_occid.find(md[i].name) != sensors_by_occid.end()) {
            for (const auto& sensor : sensors_by_occid[md[i].name]) {
                if (sensor.socket_num != socket_num) {
                    // skip all sensors not on current chip
                    continue;
                }

                // declare vars here (declarations inside switch-case are forbidden/ugly)
                uint32_t scale, freq;
                uint64_t sample, energy, timestamp, update_tag;

                switch(sensor.type) {
                case occ_sensor_sample_type::sample:
                    // regular sensor
                    scale = be32toh(md[i].scale_factor);
                    sample = get_sample(hb, &md[i]);
                    values_by_sensor[sensor].fill((double) (sample * get_occ_scale_as_fp(scale)));
                    break;

                case occ_sensor_sample_type::acc:
                    // accumulator
                    energy = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_ACCUMULATOR);
                    freq = be32toh(md[i].freq);
                    values_by_sensor[sensor].fill((double) (energy / get_occ_scale_as_fp(freq)));
                    break;

                case occ_sensor_sample_type::acc_raw:
                    // raw accumulator value
                    energy = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_ACCUMULATOR);
                    values_by_sensor[sensor].fill((uint64_t) energy);
                    break;

                case occ_sensor_sample_type::acc_raw_freq:
                    // reported sampling frequency for accumularor
                    freq = get_occ_scale_as_fp(be32toh(md[i].freq));
                    values_by_sensor[sensor].fill((double) freq);
                    break;

                case occ_sensor_sample_type::timestamp:
                    // timestamp
                    timestamp = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_TIMESTAMP);
                    values_by_sensor[sensor].fill((uint64_t) timestamp);
                    break;

                case occ_sensor_sample_type::update_tag:
                    update_tag = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_UPDATE_TAG);
                    values_by_sensor[sensor].fill((uint64_t) update_tag);
                    break;

                case occ_sensor_sample_type::acc_derivative:
                    update_tag = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_UPDATE_TAG);
                    energy = read_occ_sensor(hb, be32toh(md[i].reading_offset), SENSOR_ACCUMULATOR);
                    freq = be32toh(md[i].freq);
                    values_by_sensor[sensor].acc_raw = energy;
                    values_by_sensor[sensor].update_tag = update_tag;
                    values_by_sensor[sensor].acc_freq = get_occ_scale_as_fp(freq);
                    break;
                }
            }
        }
    }

    return values_by_sensor;
}

std::map<occ_sensor_t, all_sample_data> get_sensor_values(void* buf, const std::set<occ_sensor_t>& requested_sensors, const int socket_count)
{
    std::map<occ_sensor_t, all_sample_data> values_by_sensor;

    // iterate over all sockets
    for (size_t socket_num = 0; socket_num < socket_count; socket_num++) {
        for (const auto it : get_sensor_values_single_socket(buf, requested_sensors, socket_num)) {
            values_by_sensor[it.first] = it.second;
        }
    }

    if (requested_sensors.size() != values_by_sensor.size()) {
        throw std::runtime_error(
            "could not find all sensors in extraced file (expected: " +
            std::to_string(requested_sensors.size()) +
            ", got: " + std::to_string(values_by_sensor.size()) + ")");
    }

    return values_by_sensor;
}
