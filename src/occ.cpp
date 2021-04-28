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
#include <occ.hpp>

#include <cmath>
#include <cstdint>
#include <endian.h>
#include <stdexcept>

unsigned long read_occ_sensor(struct occ_sensor_data_header* hb, uint32_t offset, int attr)
{
    struct occ_sensor_record* sensor =
        get_sensor_addr<struct occ_sensor_record>(hb, offset);
    if (nullptr == sensor) {
        throw std::runtime_error(
            "error during sensor reading: neither ping nor pong buffer ready");
    }

    switch (attr) {
    case SENSOR_SAMPLE:
        return be16toh(sensor->sample);
    case SENSOR_ACCUMULATOR:
        return be64toh(sensor->accumulator);
    case SENSOR_TIMESTAMP:
        return be64toh(sensor->timestamp);
    case SENSOR_UPDATE_TAG:
        return be32toh(sensor->update_tag);
    }

    throw std::runtime_error("invalid sensor attribute: " + std::to_string(attr));
}

unsigned long read_occ_counter(struct occ_sensor_data_header* hb, uint32_t offset)
{
    struct occ_sensor_counter* sensor =
        get_sensor_addr<struct occ_sensor_counter>(hb, offset);
    if (nullptr == sensor) {
        throw std::runtime_error(
            "error during sensor reading: neither ping nor pong buffer ready");
    }

    return be64toh(sensor->accumulator);
}

double get_occ_scale_as_fp(uint32_t f)
{
    return (f >> 8) * pow(10, ((int8_t)(f & 0xFF)));
}

uint64_t get_sample(struct occ_sensor_data_header* header_buffer, struct occ_sensor_name* md)
{
    uint32_t offset = be32toh(md->reading_offset);
    if (md->structure_type == OCC_SENSOR_READING_FULL) {
        return read_occ_sensor(header_buffer, offset, SENSOR_SAMPLE);
    }
    else {
        return read_occ_counter(header_buffer, offset);
    }
}
