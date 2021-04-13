#include <occ.hpp>

#include <cmath>
#include <cstdint>
#include <endian.h>
#include <stdexcept>

unsigned long read_occ_sensor(struct occ_sensor_data_header *hb, uint32_t offset, int attr) {
	struct occ_sensor_record *sensor = get_sensor_addr<struct occ_sensor_record>(hb, offset);
    if (nullptr == sensor) {
		throw std::runtime_error("error during sensor reading: neither ping nor pong buffer ready");
	}

	switch (attr) {
	case SENSOR_SAMPLE:
		return be16toh(sensor->sample);
	case SENSOR_ACCUMULATOR:
		return be64toh(sensor->accumulator);
	}

    throw std::runtime_error("invalid sensor attribute: " + std::to_string(attr));
}

unsigned long read_occ_counter(struct occ_sensor_data_header *hb, uint32_t offset) {
	struct occ_sensor_counter *sensor = get_sensor_addr<struct occ_sensor_counter>(hb, offset);
    if (nullptr == sensor) {
		throw std::runtime_error("error during sensor reading: neither ping nor pong buffer ready");
	}

	return be64toh(sensor->accumulator);
}

double get_occ_scale_as_fp(uint32_t f) {
    return (f >> 8) * pow(10, ((int8_t)(f & 0xFF)));
}

uint64_t get_sample(struct occ_sensor_data_header* header_buffer, struct occ_sensor_name* md) {
    uint32_t offset = be32toh(md->reading_offset);
    if (md->structure_type == OCC_SENSOR_READING_FULL) {
        return read_occ_sensor(header_buffer, offset, SENSOR_SAMPLE);
    } else {
        return read_occ_counter(header_buffer, offset);
    }
}
