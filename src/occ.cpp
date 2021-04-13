#include <occ.hpp>

#include <cmath>
#include <cstdint>
#include <endian.h>
#include <stdexcept>

unsigned long read_occ_sensor(struct occ_sensor_data_header *hb, uint32_t offset, int attr) {
    struct occ_sensor_record *sping, *spong;
	struct occ_sensor_record *sensor = NULL;
	uint8_t *ping, *pong;

	ping = (uint8_t *)((uint64_t)hb + be32toh(hb->reading_ping_offset));
	pong = (uint8_t *)((uint64_t)hb + be32toh(hb->reading_pong_offset));
	sping = (struct occ_sensor_record *)((uint64_t)ping + offset);
	spong = (struct occ_sensor_record *)((uint64_t)pong + offset);

	if (*ping && *pong) {
		if (be64toh(sping->timestamp) > be64toh(spong->timestamp))
			sensor = sping;
		else
			sensor = spong;
	} else if (*ping && !*pong) {
		sensor = sping;
	} else if (!*ping && *pong) {
		sensor = spong;
	} else if (!*ping && !*pong) {
		throw std::runtime_error("error during sensor reading: neither ping nor pong buffer ready");
	}

	switch (attr) {
	case SENSOR_SAMPLE:
		return be16toh(sensor->sample);
	case SENSOR_ACCUMULATOR:
		return be64toh(sensor->accumulator);
	default:
		break;
	}

	return 0;
}

unsigned long read_occ_counter(struct occ_sensor_data_header *hb, uint32_t offset) {
	struct occ_sensor_counter *sping, *spong;
	struct occ_sensor_counter *sensor = NULL;
	uint8_t *ping, *pong;

	ping = (uint8_t *)((uint64_t)hb + be32toh(hb->reading_ping_offset));
	pong = (uint8_t *)((uint64_t)hb + be32toh(hb->reading_pong_offset));
	sping = (struct occ_sensor_counter *)((uint64_t)ping + offset);
	spong = (struct occ_sensor_counter *)((uint64_t)pong + offset);

	if (*ping && *pong) {
		if (be64toh(sping->timestamp) > be64toh(spong->timestamp))
			sensor = sping;
		else
			sensor = spong;
	} else if (*ping && !*pong) {
		sensor = sping;
	} else if (!*ping && *pong) {
		sensor = spong;
	} else if (!*ping && !*pong) {
		throw std::runtime_error("error during sensor reading: neither ping nor pong buffer ready");
	}

	return be64toh(sensor->accumulator);
}

double get_occ_scale_as_fp(uint32_t f) {
    return (f >> 8) * pow(10, ((int8_t)(f & 0xFF)));
}
