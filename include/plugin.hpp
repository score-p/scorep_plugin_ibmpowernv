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
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>

#include <scorep/SCOREP_MetricTypes.h>
#include <scorep/plugin/plugin.hpp>

#include <occ_sensor_t.hpp>
#include <occ.hpp>
#include <occ_util.hpp>
#include <util.hpp>

// TODO apply clang format

using scorep::plugin::log::logging;


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
        sampling_interval = get_ns_from_str(scorep::environment_variable::get("INTERVAL", "10ms"));

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

        logging::debug() << "adding sensors for pattern \"" << pattern << "\"";

        if ("*" == pattern) {
            // return all metrics
            for (const auto& it : occ_sensor_t::metric_properties_by_sensor) {
                make_handle(it.second.name, it.first.name, it.first.acc);
                result.push_back(it.second);
            }
            return result;
        }

        // create map: metric name -> sensor object (which will in turn be mapped to a metric object)
        // Note: I would love to be the value type a metric_property directly, BUT THE WRAPPER IS NOT SPECIFIED TO SUPPORT THAT
        std::map<std::string, occ_sensor_t> occ_sensors_by_name;
        for (const auto& it : occ_sensor_t::metric_properties_by_sensor) {
            occ_sensors_by_name[it.second.name] = it.first;
        }

        // iterate over items in comma-seperated list
        std::stringstream comma_separated_list(pattern);
        std::string list_entry;
        while(std::getline(comma_separated_list, list_entry, ',')) {
            if (occ_sensors_by_name.find(list_entry) == occ_sensors_by_name.end()) {
                logging::error() << "unkown metric: " << list_entry;
                continue;
            }
            
            auto occ_sensor = occ_sensors_by_name.at(list_entry);
            auto metric_property = occ_sensor_t::metric_properties_by_sensor.at(occ_sensor);
            make_handle(metric_property.name, occ_sensor.name, occ_sensor.acc);
            result.push_back(metric_property);
        }

        return result;
    }

    void add_metric(const occ_sensor_t& sensor)
    {
        check_fatal();
        logging::debug() << "adding sensor for recording: " << sensor.name;

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

        if (value_buffers_by_sensor.find(sensor) == value_buffers_by_sensor.end()) {
            logging::error() << "sensor not recorded: " << sensor.name;
            throw std::runtime_error("unkown sensor requested");
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
