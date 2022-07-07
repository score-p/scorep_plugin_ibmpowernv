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

#include <sys/syscall.h>
#include <sys/types.h>

#include <scorep/SCOREP_MetricTypes.h>
#include <scorep/plugin/plugin.hpp>

#include <occ.hpp>
#include <occ_sensor_t.hpp>
#include <occ_util.hpp>
#include <util.hpp>

using scorep::plugin::log::logging;

class ibmpowernv_sync_plugin
    : public scorep::plugin::base<ibmpowernv_sync_plugin,
                                  scorep::plugin::policy::sync_strict,
                                  scorep::plugin::policy::scorep_clock,
                                  scorep::plugin::policy::per_thread,
                                  scorep::plugin::policy::synchronize,
                                  occ_metric_policy> {
public:
    ibmpowernv_sync_plugin();

    std::vector<scorep::plugin::metric_property> get_metric_properties(const std::string& pattern);
    void add_metric(const occ_metric_t& metric);
    template <typename Proxy>
    void get_current_value(const occ_metric_t& metric, Proxy& p);
    void synchronize(bool is_responsible, SCOREP_MetricSynchronizationMode sync_mode);

private:
    /// indicates that a fatal error has occured and that all operation should be seized
    std::atomic<bool> fatal_occured = false;

    /// minimal duration between measurments. Shorter request use cached data
    std::chrono::nanoseconds min_measurment_interval;

    /// file descriptor for the opened occ_inband_sensors file
    int occ_file_fd;
    /// all sensors
    std::set<occ_sensor_t> requested_sensors_set;

    // initial measurement data
    std::map<occ_sensor_t, sensor_data_t> init_measurement_data;
    std::chrono::steady_clock::time_point init_measurement =
        std::chrono::steady_clock::now();

    /// last measurment data, only filled for acc_derivative metrics
    std::map<occ_sensor_t, sensor_data_t> last_measurement_data;

    /// timestamp of current_measurement_data
    std::chrono::steady_clock::time_point current_measurement = std::chrono::steady_clock::now();
    /// current measurment data, used for returning cached results
    std::map<occ_sensor_t, sensor_data_t> current_measurement_data;

    /// number of sockets to read from
    std::atomic<int> socket_count = 2;

    /// hostname of current machine, used for identification
    std::string hostname;
    /// true iff this instance is of the **rank** responsible to record data for this host
    bool is_resposible = false;
    /// if this *rank* is responsible: ID of the thread responsible for data recording
    pid_t responsible_thread = -1;

    bool metric_properties_added = false;

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
    }

    /// seeks sensors file & copies it into given buffer
    void read_file_into_buffer(void* buf, int socket_count);
};
