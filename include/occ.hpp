#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__

#include <cstdint>

#define MAX_OCCS			8
#define MAX_CHARS_SENSOR_NAME		16
#define MAX_CHARS_SENSOR_UNIT		4

#define OCC_SENSOR_DATA_BLOCK_OFFSET	0x00580000
#define OCC_SENSOR_DATA_BLOCK_SIZE	0x00025800

enum occ_sensor_type {
	OCC_SENSOR_TYPE_GENERIC		= 0x0001,
	OCC_SENSOR_TYPE_CURRENT		= 0x0002,
	OCC_SENSOR_TYPE_VOLTAGE		= 0x0004,
	OCC_SENSOR_TYPE_TEMPERATURE	= 0x0008,
	OCC_SENSOR_TYPE_UTILIZATION	= 0x0010,
	OCC_SENSOR_TYPE_TIME		= 0x0020,
	OCC_SENSOR_TYPE_FREQUENCY	= 0x0040,
	OCC_SENSOR_TYPE_POWER		= 0x0080,
	OCC_SENSOR_TYPE_PERFORMANCE	= 0x0200,
};

enum occ_sensor_location {
	OCC_SENSOR_LOC_SYSTEM		= 0x0001,
	OCC_SENSOR_LOC_PROCESSOR	= 0x0002,
	OCC_SENSOR_LOC_PARTITION	= 0x0004,
	OCC_SENSOR_LOC_MEMORY		= 0x0008,
	OCC_SENSOR_LOC_VRM		= 0x0010,
	OCC_SENSOR_LOC_OCC		= 0x0020,
	OCC_SENSOR_LOC_CORE		= 0x0040,
	OCC_SENSOR_LOC_GPU		= 0x0080,
	OCC_SENSOR_LOC_QUAD		= 0x0100,
};

enum sensor_struct_type {
	OCC_SENSOR_READING_FULL		= 0x01,
	OCC_SENSOR_READING_COUNTER	= 0x02,
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
unsigned long read_occ_sensor(struct occ_sensor_data_header *hb, uint32_t offset, int attr);

/**
 * read counter from given buffer at given offset.
 * handles ping/pong buffer selection and endian conversion.
 * generally used when other sensors/accumulators overflow.
 * @param hb buffer to read from
 * @param offset offset to read at
 * @return value of the given counter, NOT YET SCALED
 * @throws std::runtime_error of neither ping nor pong buffer is ready
 */
unsigned long read_occ_counter(struct occ_sensor_data_header *hb, uint32_t offset);

/**
 * convert occ-internal format to FP value.
 * used to convert the scaling factors
 * @param f raw value from occ
 * @return interpreted number
 */
double get_occ_scale_as_fp(uint32_t f);

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_HPP_INCLUDED__
