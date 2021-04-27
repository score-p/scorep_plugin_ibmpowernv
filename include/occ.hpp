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
#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__

#include <cstdint>
#include <endian.h>

#define MAX_OCCS 8
#define MAX_CHARS_SENSOR_NAME 16
#define MAX_CHARS_SENSOR_UNIT 4

#define OCC_SENSOR_DATA_BLOCK_OFFSET 0x00580000
#define OCC_SENSOR_DATA_BLOCK_SIZE 0x00025800

enum occ_sensor_type {
    OCC_SENSOR_TYPE_GENERIC = 0x0001,
    OCC_SENSOR_TYPE_CURRENT = 0x0002,
    OCC_SENSOR_TYPE_VOLTAGE = 0x0004,
    OCC_SENSOR_TYPE_TEMPERATURE = 0x0008,
    OCC_SENSOR_TYPE_UTILIZATION = 0x0010,
    OCC_SENSOR_TYPE_TIME = 0x0020,
    OCC_SENSOR_TYPE_FREQUENCY = 0x0040,
    OCC_SENSOR_TYPE_POWER = 0x0080,
    OCC_SENSOR_TYPE_PERFORMANCE = 0x0200,
};

enum occ_sensor_location {
    OCC_SENSOR_LOC_SYSTEM = 0x0001,
    OCC_SENSOR_LOC_PROCESSOR = 0x0002,
    OCC_SENSOR_LOC_PARTITION = 0x0004,
    OCC_SENSOR_LOC_MEMORY = 0x0008,
    OCC_SENSOR_LOC_VRM = 0x0010,
    OCC_SENSOR_LOC_OCC = 0x0020,
    OCC_SENSOR_LOC_CORE = 0x0040,
    OCC_SENSOR_LOC_GPU = 0x0080,
    OCC_SENSOR_LOC_QUAD = 0x0100,
};

enum sensor_struct_type {
    OCC_SENSOR_READING_FULL = 0x01,
    OCC_SENSOR_READING_COUNTER = 0x02,
};

struct occ_sensor_record {
    uint16_t gsid;
    uint64_t timestamp;
    uint16_t sample;
    uint16_t sample_min;
    uint16_t sample_max;
    uint16_t csm_min;
    uint16_t csm_max;
    uint16_t profiler_min;
    uint16_t profiler_max;
    uint16_t job_scheduler_min;
    uint16_t job_scheduler_max;
    uint64_t accumulator;
    uint32_t update_tag;
    uint8_t pad[8];
} __attribute__((__packed__));
struct occ_sensor_counter {
    uint16_t gsid;
    uint64_t timestamp;
    uint64_t accumulator;
    uint8_t sample;
    uint8_t pad[5];
} __attribute__((__packed__));

struct occ_sensor_data_header {
    uint8_t valid;
    uint8_t version;
    uint16_t nr_sensors;
    uint8_t reading_version;
    uint8_t pad[3];
    uint32_t names_offset;
    uint8_t names_version;
    uint8_t name_length;
    uint16_t reserved;
    uint32_t reading_ping_offset;
    uint32_t reading_pong_offset;
} __attribute__((__packed__));
struct occ_sensor_name {
    char name[MAX_CHARS_SENSOR_NAME];
    char units[MAX_CHARS_SENSOR_UNIT];
    uint16_t gsid;
    uint32_t freq;
    uint32_t scale_factor;
    uint16_t type;
    uint16_t location;
    uint8_t structure_type;
    uint32_t reading_offset;
    uint8_t sensor_data;
    uint8_t pad[8];
} __attribute__((__packed__));
enum sensor_attr {
    SENSOR_SAMPLE,
    SENSOR_ACCUMULATOR,
    SENSOR_TIMESTAMP,
};

/**
 * read sensor from given buffer at given offset.
 * handles ping/pong buffer selection and endian conversion.
 * @param hb buffer to read from
 * @param offset offset to read at
 * @param attr type of sensor, e.g. accumulator
 * @return value of the given sensor, NOT YET SCALED
 * @throws std::runtime_error of neither ping nor pong buffer is ready
 */
unsigned long read_occ_sensor(struct occ_sensor_data_header* hb, uint32_t offset, int attr);

/**
 * read counter from given buffer at given offset.
 * handles ping/pong buffer selection and endian conversion.
 * generally used when other sensors/accumulators overflow.
 * @param hb buffer to read from
 * @param offset offset to read at
 * @return value of the given counter, NOT YET SCALED
 * @throws std::runtime_error of neither ping nor pong buffer is ready
 */
unsigned long read_occ_counter(struct occ_sensor_data_header* hb, uint32_t offset);

/**
 * extract sample with given name struct from given buffer.
 * automatically selects if read_occ_sensor or read_occ_counter should be used.
 * @param header_buffer buffer to be read
 * @param md name struct for the desired sensor
 * @return sensor value NOT YET SCALED
 */
uint64_t get_sample(struct occ_sensor_data_header* header_buffer, struct occ_sensor_name* md);

/**
 * convert occ-internal format to FP value.
 * used to convert the scaling factors
 * @param f raw value from occ
 * @return interpreted number
 */
double get_occ_scale_as_fp(uint32_t f);

/**
 * get the address of the sensor at the given offset,
 * selects active ping/pong buffer
 * @param hb buffer to read from
 * @param offset desired offset to read
 * @return ptr to buffer at desired offset; nullptr if no buffer is valid
 */
template <typename T>
T* get_sensor_addr(struct occ_sensor_data_header* hb, uint32_t offset)
{
    T *sping, *spong;
    uint8_t *ping, *pong;

    ping = (uint8_t*)((uint64_t)hb + be32toh(hb->reading_ping_offset));
    pong = (uint8_t*)((uint64_t)hb + be32toh(hb->reading_pong_offset));
    sping = (T*)((uint64_t)ping + offset);
    spong = (T*)((uint64_t)pong + offset);

    if (*ping && *pong) {
        // both valid -> pick newer
        if (be64toh(sping->timestamp) > be64toh(spong->timestamp)) {
            return sping;
        }
        else {
            return spong;
        }
    }
    else if (*ping && !*pong) {
        return sping;
    }
    else if (!*ping && *pong) {
        return spong;
    }

    // no buffer valid
    return nullptr;
}

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__
