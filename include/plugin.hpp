#include <scorep/plugin/plugin.hpp>

#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <endian.h>
#include <scorep/SCOREP_MetricTypes.h>

// TODO apply clang format

using scorep::plugin::log::logging;
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

static unsigned long read_occ_sensor(struct occ_sensor_data_header *hb, uint32_t offset, int attr) {
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
		return 0;
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
		return 0;
	}

	return be64toh(sensor->accumulator);
}

/// contains all information required to locate a sensor and grab its value
struct occ_sensor_t {
    occ_sensor_t(const std::string& name, const bool acc) : name(name), acc(acc) {}

    /// name as used in occ_inband_sensors (e.g. PWRSYS)
    std::string name;
    /// whether to use the accumulator
    bool acc = false;

    /// metric properties for all supported sensors
    static const std::map<occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor;
};

// somehow intializer lists fail TODO investigate
const std::map<occ_sensor_t, scorep::plugin::metric_property> occ_sensor_t::metric_properties_by_sensor = {
    {
        {"PWRSYS", false},
        scorep::plugin::metric_property("power_system", "power intake of the entire system", "W")
        //{
        //    "power_system",
        //    "power intake of the entire system",
        //    "W",
        //    SCOREP_METRIC_MODE_ABSOLUTE_POINT,
        //    SCOREP_METRIC_VALUE_DOUBLE,
        //    SCOREP_METRIC_BASE_DECIMAL,
        //    0
        //}
    },
    {
        {"PWRSYS", true},
        scorep::plugin::metric_property("power_system_acc", "accumulator of consumed energy by entire system", "J")
        //  {
        //      "power_system_acc",
        //      "accumulator of consumed energy by entire system",
        //      "J",
        //      SCOREP_METRIC_MODE_ACCUMULATED_POINT,
        //      SCOREP_METRIC_VALUE_DOUBLE,
        //      SCOREP_METRIC_BASE_DECIMAL,
        //      0
        //  }
    },
};

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    if (lhs.name != rhs.name) {
        return lhs.name < rhs.name;
    }

    return lhs.acc < rhs.acc;
}

template <typename T, typename Policies>
using occ_sensor_policy = scorep::plugin::policy::object_id<occ_sensor_t, T, Policies>;

static std::chrono::nanoseconds get_ns_from_str(const std::string& interval_str, const std::chrono::nanoseconds default_value = std::chrono::milliseconds(10))
{
    std::regex r("([0-9]+)([mun]?s)");
    std::smatch s;

    if (std::regex_match(interval_str, s, r)) {
        std::string time = s[1];
        std::string unit = s[2];

        switch (unit[0]) {
        case 's':
            return std::chrono::seconds(std::stol(time));
        case 'm':
            return std::chrono::milliseconds(std::stol(time));
        case 'u':
            return std::chrono::microseconds(std::stol(time));
        case 'n':
            return std::chrono::nanoseconds(std::stol(time));
        default:
            logging::fatal() << "regex parsing got exploited";
            break;
        }
    }

    return default_value;
}

/// create map with OCC identifier (string like "PWRSYS") -> sensor(s)
std::map<std::string, std::set<occ_sensor_t>> get_sensors_by_occid(const std::set<occ_sensor_t>& sensors) {
    std::map<std::string, std::set<occ_sensor_t>> m;
    for (const auto& s : sensors) {
        m[s.name].insert(s);
    }
    return m;
}

/// extract sample from given buffer
uint64_t get_sample(struct occ_sensor_data_header* header_buffer, struct occ_sensor_name* md) {
    uint32_t offset = be32toh(md->reading_offset);
    if (md->structure_type == OCC_SENSOR_READING_FULL) {
        return read_occ_sensor(header_buffer, offset, SENSOR_SAMPLE);
    } else {
        return read_occ_counter(header_buffer, offset);
    }
}

/// convert occ-internal format to FP value
static double get_occ_scale_as_fp(uint32_t f) {
    return (f >> 8) * pow(10, ((int8_t)(f & 0xFF)));
}

/// extract values from occ inband sensors file for given sensors
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
        logging::error() << "could not find all sensors in extraced file (expected: " << requested_sensors.size() << ", got: " << values_by_sensor.size() << ")";
        throw std::runtime_error("requested more sensors than available in occ file");
    }

    return values_by_sensor;
}

class ibmpowernv_plugin : public scorep::plugin::base<ibmpowernv_plugin,
                                                      scorep::plugin::policy::async,
                                                      scorep::plugin::policy::post_mortem,
                                                      scorep::plugin::policy::scorep_clock,
                                                      scorep::plugin::policy::per_host,
                                                      occ_sensor_policy> {
public:
    /// id of the chip to be used; testing suggests leaving it at 0 is best
    int chipid = 0;
    
    ibmpowernv_plugin()
    {
        sampling_interval = get_ns_from_str(scorep::environment_variable::get("INTERVAL", "10ms"), std::chrono::milliseconds(10));

        occ_file_fd = open(scorep::environment_variable::get("OCC_FILENAME", "/sys/firmware/opal/exports/occ_inband_sensors").c_str(), O_RDONLY);
        if (occ_file_fd < 0) {
            fatal("could not open sensor file");
        }

        logging::info() << "ibmpowernv plugin constructed";
    }

    std::vector<scorep::plugin::metric_property> get_metric_properties(const std::string& pattern)
    {
        check_fatal();

        std::vector<scorep::plugin::metric_property> result;
        std::regex search_regex(pattern);

        logging::debug() << "adding sensors for pattern \"" << pattern << "\"";

        // iterate over all sensors
        for (const auto it : occ_sensor_t::metric_properties_by_sensor) {
            if (regex_match(it.second.name, search_regex)) {
                result.push_back(it.second);
                logging::trace() << "added sensor " << it.second.description;
            }
        }

        return result;
    }

    void add_metric(const occ_sensor_t& sensor)
    {
        check_fatal();

        // we just got notified that the given sensor will be recorded
        // -> init an empty buffer for it
        value_buffers_by_sensor[sensor] = {};
    }

    void start()
    {
        check_fatal();

        if (running) {
            return;
        }

        running = true;
        last_measurement_ = std::chrono::system_clock::now();
        collection_thread = std::thread([&]() { this->exec(); });
        logging::info() << "started recording of " << value_buffers_by_sensor.size() << " sensors";
    }

    void stop()
    {
        check_fatal();

        if (!running) {
            return;
        }

        running = false;
        collection_thread.join();
    }

    void exec()
    {
        check_fatal();
        
        std::shared_ptr<void> buf(new int8_t[OCC_SENSOR_DATA_BLOCK_SIZE]);
        if (nullptr == buf) {
            fatal("couldn't allocate buffer for occ sensor data");
        }

        // requested sensors: all keys that have been initialized (with an empty vector)
        std::set<occ_sensor_t> requested_sensors_set;
        std::transform(value_buffers_by_sensor.begin(),
                       value_buffers_by_sensor.end(),
                       std::inserter(requested_sensors_set, requested_sensors_set.end()),
                       [](auto pair){return pair.first;}
                       );

        while (running) {
            check_fatal();

            // read occ file
            times_.push_back(scorep::chrono::measurement_clock::now());
            read_file_into_buffer(buf.get());

            // parse data from file & store values
            for (const auto& it : get_sensor_values(buf.get(), requested_sensors_set)) {
                value_buffers_by_sensor[it.first].push_back(it.second);
            }

            // skip forward to next measurement point
            while (last_measurement_ < std::chrono::system_clock::now()) {
                last_measurement_ += sampling_interval;
            }
            std::this_thread::sleep_until(last_measurement_);
        }
    }

    template <typename Cursor>
    void get_all_values(const occ_sensor_t& sensor, Cursor& c)
    {
        check_fatal();

        if (running) {
            fatal("can't extract values while plugin is running, will produce duplicate data points");
        }

        if (times_.size() != value_buffers_by_sensor.at(sensor).size()) {
            logging::error() << "check failed: value not for each time point recorded";
            throw std::runtime_error("can't associate recorded values -> time points");
        }
        
        // write buffered measurements for given sensor
        for (int i = 0; i < times_.size(); i++) {
            c.write(times_[i], value_buffers_by_sensor[sensor][i]);
        }
    }

private:
    /// indicates that a fatal error has occured and that all operation should be seized
    std::atomic<bool> fatal_occured = false;
    /// indicates if the collection_thread should stop working
    std::atomic<bool> running = false;
    /// thread that will collect actual measurements
    std::thread collection_thread;
    /// points in time when measurements have been taken
    std::vector<scorep::chrono::ticks> times_;
    /// interval at which the collection_thread should make measurements
    std::chrono::nanoseconds sampling_interval;
    /// TODO change type (WTF wall clock time?!)
    std::chrono::system_clock::time_point last_measurement_ = std::chrono::system_clock::now();
    /// file descriptor for the opened occ_inband_sensors file
    int occ_file_fd;
    /// recorded values for each sensor
    std::map<occ_sensor_t, std::vector<double>> value_buffers_by_sensor;

    /// throws if a fatal condition has occured
    void check_fatal() {
        if (fatal_occured) {
            logging::warn() << "OCC driver has been called, but a fatal error has occured";
            throw std::logic_error("OCC driver called after fatal condition");
        }
    }

    void fatal(const std::string msg) {
        fatal_occured = true;
        logging::fatal() << "[!!] " << msg;
        throw std::runtime_error(msg);
    }

    /// seeks sensors file & copies it into given buffer
    void read_file_into_buffer(void* buf) {
        // read data from file to buffer
        lseek(occ_file_fd, chipid * OCC_SENSOR_DATA_BLOCK_SIZE, SEEK_SET);
        int rc, bytes;
        for (bytes = 0; bytes < OCC_SENSOR_DATA_BLOCK_SIZE; bytes += rc) {
            rc = read(occ_file_fd, (int8_t*) buf + bytes, OCC_SENSOR_DATA_BLOCK_SIZE - bytes);
            if (0 == rc) {
                fatal("read syscall on occ_sensors file returned 0 bytes");
            }
            if (rc < 0) {
                fatal("failed to read from occ_sensors_file");
            }
        }

        // check if read successfull
        if (bytes != OCC_SENSOR_DATA_BLOCK_SIZE) {
            fatal("read from occ_sensors_file finished, but did not read enough");
        }
    }
};
