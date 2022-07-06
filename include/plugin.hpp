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
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <scorep/SCOREP_MetricTypes.h>
#include <scorep/plugin/plugin.hpp>

#include <occ.hpp>
#include <occ_sensor_t.hpp>
#include <occ_util.hpp>
#include <util.hpp>

using scorep::plugin::log::logging;

class ibmpowernv_plugin
    : public scorep::plugin::base<ibmpowernv_plugin,
                                  scorep::plugin::policy::async,
                                  scorep::plugin::policy::post_mortem,
                                  scorep::plugin::policy::scorep_clock,
                                  scorep::plugin::policy::per_host,
                                  occ_metric_policy> {
public:
    ibmpowernv_plugin()
    {
        sampling_interval = get_ns_from_str(
            scorep::environment_variable::get("INTERVAL", "10ms"));

        occ_file_fd = open(
            scorep::environment_variable::get(
                "OCC_FILENAME", "/sys/firmware/opal/exports/occ_inband_sensors")
                .c_str(),
            O_RDONLY);
        if (occ_file_fd < 0) {
            fatal("could not open sensor file");
        }

        try {
            socket_count = std::stoi(scorep::environment_variable::get("SOCKETS", "2"));
        } catch (const std::invalid_argument&) {
            fatal("could not determine number of sockets to read");
        }
        if (socket_count < 1) {
            fatal("must at least watch one socket");
        }

        logging::info() << "ibmpowernv plugin constructed";
    }

    std::vector<scorep::plugin::metric_property> get_metric_properties(const std::string& pattern)
    {
        check_fatal();

        if ("*" != pattern) {
            fatal("only supported pattern is '*'");
        }

        std::vector<scorep::plugin::metric_property> result;

        // retrieve all possible metrics
        for (unsigned socket = 0; socket < socket_count; socket++) {
            auto occ_sensor_types = occ_power_sensor_type_all_occs;
            if (0 == socket) {
                occ_sensor_types.insert(occ_power_sensor_type_master_only.begin(),
                                        occ_power_sensor_type_master_only.end());
            }

            logging::debug() << "socket " << socket << ": inserting metrics for " << occ_sensor_types.size() << " sensors";

            for (const auto& sensor_type : occ_sensor_types) {
                // build metrics for this sensor
                occ_sensor_t sensor{sensor_type, socket};

                for (const auto sample_type : occ_sensor_sample_type_common) {
                    // metric to be recorded into trace
                    occ_metric_t metric{sensor, sample_type};

                    // result contains metric properties
                    make_handle(metric.get_name(), metric);
                    result.push_back(metric.get_metric_property());
                }
            }
        }
        
        return result;
    }

    void add_metric(const occ_metric_t& metric)
    {
        check_fatal();
        logging::debug() << "adding metric for recording: " << metric.get_name();

        // we just got notified that the given sensor will be recorded
        // -> init an empty buffer for it
        value_buffers_by_sensor[metric.sensor] = {};
    }

    void start()
    {
        check_fatal();

        if (running) {
            return;
        }

        running = true;
        last_measurement_ = std::chrono::steady_clock::now();
        collection_thread = std::thread([&]() { this->exec(); });
        logging::info() << "started recording of "
                        << value_buffers_by_sensor.size() << " sensors";
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
        convert.synchronize_point();

        // hold data read from sysfs file, ordered by socket
        std::shared_ptr<void> raw_occ_buffer(new int8_t[OCC_SENSOR_DATA_BLOCK_SIZE * socket_count]);
        if (nullptr == raw_occ_buffer) {
            fatal("couldn't allocate buffer for occ sensor data");
        }

        // extract keys of requested sensors map
        // (initialized keys are requested)
        std::set<occ_sensor_t> requested_sensors_set;
        std::transform(
            value_buffers_by_sensor.begin(), value_buffers_by_sensor.end(),
            std::inserter(requested_sensors_set, requested_sensors_set.end()),
            [](auto pair) { return pair.first; });

        while (running) {
            check_fatal();

            // read occ file
            times_.push_back(scorep::chrono::measurement_clock::now());
            system_times_.push_back(std::chrono::steady_clock::now());
            read_file_into_buffer(raw_occ_buffer.get(), socket_count);

            // parse data from file & store values
            for (const auto& it : get_sensor_data(raw_occ_buffer.get(), requested_sensors_set, socket_count)) {
                value_buffers_by_sensor[it.first].push_back(it.second);
            }

            // skip forward to next measurement point
            while (last_measurement_ < std::chrono::steady_clock::now()) {
                last_measurement_ += sampling_interval;
            }
            std::this_thread::sleep_until(last_measurement_);
        }
        convert.synchronize_point();
    }

    template <typename Cursor>
    void get_all_values(const occ_metric_t& metric, Cursor& c)
    {
        legacy_occ_sensor_t sensor;
        check_fatal();

        if (running) {
            fatal("can't extract values while plugin is running, will produce "
                  "duplicate data points");
        }

        if (value_buffers_by_sensor.find(metric.sensor) == value_buffers_by_sensor.end()) {
            logging::error() << "sensor not recorded: " << sensor.name;
            throw std::runtime_error("unkown sensor requested");
        }

        if (times_.size() != value_buffers_by_sensor.at(metric.sensor).size()) {
            logging::error()
                << "check failed: value not for each time point recorded";
            throw std::runtime_error(
                                     "can't associate recorded values -> time points");
        }

        if (occ_sensor_sample_type::sample == metric.sample_type) {
            for (int i = 0; i < times_.size(); i++) {
                c.write(times_[i], value_buffers_by_sensor[metric.sensor][i].sample);
            }
        }

        if (occ_sensor_sample_type::timestamp == metric.sample_type ||
            occ_sensor_sample_type::update_tag == metric.sample_type) {
            for (int i = 0; i < times_.size(); i++) {
                // set relative to first value
                switch (metric.sample_type) {
                case occ_sensor_sample_type::timestamp:
                    c.write(times_[i],
                            value_buffers_by_sensor[metric.sensor][i].timestamp - value_buffers_by_sensor[metric.sensor][0].timestamp);
                    break;
                    
                case occ_sensor_sample_type::update_tag:
                    c.write(times_[i],
                            value_buffers_by_sensor[metric.sensor][i].update_tag - value_buffers_by_sensor[metric.sensor][0].update_tag);
                    break;
                }
            }
        }

        // power from energy
        if (occ_sensor_sample_type::acc_derivative == metric.sample_type) {
            // derive actual value from acc
            double last_acc = value_buffers_by_sensor[metric.sensor][0].accumulator;
            for (int i = 0; i < times_.size(); i++) {
                double current_acc = value_buffers_by_sensor[metric.sensor][i].accumulator;
                if (current_acc > last_acc) {
                    // only act on change & no overflow (which happens after 2^64s = ~416 days, but never trust input)
                    // "update_tag" describes the (total) number of samples present in the acc -> extract delta
                    double delta_samples = value_buffers_by_sensor[metric.sensor][i].update_tag - value_buffers_by_sensor[metric.sensor][i-1].update_tag;
                    double power = (current_acc - last_acc) / delta_samples;
                    c.write(times_[i], power);
                }
                last_acc = current_acc;
            }
        }
    }

private:
    /// indicates that a fatal error has occured and that all operation should be seized
    std::atomic<bool> fatal_occured = false;
    /// indicates if the collection_thread should stop working
    std::atomic<bool> running = false;
    /// thread that will collect actual measurements
    std::thread collection_thread;
    /// points in time when measurements have been taken as scorep timestamps
    std::vector<scorep::chrono::ticks> times_;
    /// points in time when measurements have been taken as clock time (used for computations)
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> system_times_;

    /// interval at which the collection_thread should make measurements
    std::chrono::nanoseconds sampling_interval;
    /// time point of last measurement, used to align measurement interval
    std::chrono::steady_clock::time_point last_measurement_ =
        std::chrono::steady_clock::now();
    /// file descriptor for the opened occ_inband_sensors file
    int occ_file_fd;
    /// recorded values for each sensor
    std::map<occ_sensor_t, std::vector<sensor_data_t>> value_buffers_by_sensor;
    /// used to convert times
    scorep::chrono::time_convert<> convert;
    /// number of sockets to read from
    std::atomic<int> socket_count = 2;

    /// throws if a fatal condition has occured
    void check_fatal()
    {
        if (fatal_occured) {
            logging::warn()
                << "OCC driver has been called, but a fatal error has occured";
            throw std::logic_error("OCC driver called after fatal condition");
        }
    }

    /// throw a fatal error
    void fatal(const std::string msg)
    {
        fatal_occured = true;
        logging::fatal() << "[!!] " << msg;
        throw std::runtime_error(msg);
    }

    /// seeks sensors file & copies it into given buffer
    void read_file_into_buffer(void* buf, int socket_count)
    {
        const size_t total_to_read_bytes = socket_count * OCC_SENSOR_DATA_BLOCK_SIZE;
        // read data from file to buffer
        lseek(occ_file_fd, 0, SEEK_SET);
        int rc, bytes;
        for (bytes = 0; bytes < total_to_read_bytes; bytes += rc) {
            rc = read(occ_file_fd, (int8_t*)buf + bytes, total_to_read_bytes - bytes);
            if (0 == rc) {
                fatal("read syscall on occ_sensors file returned 0 bytes");
            }
            if (rc < 0) {
                fatal("failed to read from occ_sensors_file");
            }
        }

        // check if read successfull
        if (bytes != total_to_read_bytes) {
            fatal(
                "read from occ_sensors_file finished, but did not read enough");
        }
    }
};
