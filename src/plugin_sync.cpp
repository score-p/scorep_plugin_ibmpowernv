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
#include <iostream>
#include <plugin_sync.hpp>

#ifdef HAVE_MPI
#include <mpi.h>
#endif

ibmpowernv_sync_plugin::ibmpowernv_sync_plugin()
{
    min_measurment_interval =
        get_ns_from_str(scorep::environment_variable::get("INTERVAL", "10ms"));

    occ_file_fd = open(
        scorep::environment_variable::get(
            "OCC_FILENAME", "/sys/firmware/opal/exports/occ_inband_sensors")
            .c_str(),
        O_RDONLY);
    if (occ_file_fd < 0) {
        fatal("could not open sensor file");
    }

    try {
        socket_count =
            std::stoi(scorep::environment_variable::get("SOCKETS", "2"));
    }
    catch (const std::invalid_argument&) {
        fatal("could not determine number of sockets to read");
    }
    if (socket_count < 1) {
        fatal("must at least watch one socket");
    }

    logging::info() << "ibmpowernv_sync plugin constructed";
}

std::vector<scorep::plugin::metric_property> ibmpowernv_sync_plugin::get_metric_properties(
    const std::string& pattern)
{
    check_fatal();

    logging::debug() << "received get_metric_properties(" << pattern << ")";

    std::vector<scorep::plugin::metric_property> properties;
    if (metric_properties_added) {
        logging::debug()
            << "Metric properties already added, doing nothing.";
        return properties;
    }

    if ("*" != pattern) {
        fatal("only pattern '*' supported");
    }

    std::vector<scorep::plugin::metric_property> result;

    // global metrics (only available on first socket)
    for (const auto& it : occ_sensor_t::metric_properties_by_sensor_master_only) {
        auto& handle = make_handle(it.second.name, it.first.name, it.first.type,
                                   it.first.socket_num, it.first.quantity);
        result.push_back(it.second);
        requested_sensors_set.insert(handle);
    }

    // socket-local metrics (available on every socket)
    for (size_t socket_num = 0; socket_num < socket_count; socket_num++) {
        for (const auto& it : occ_sensor_t::metric_properties_by_sensor_per_socket) {
            auto scorep_metric = it.second;
            scorep_metric.name += "." + std::to_string(socket_num);
            // !! use socket_num from loop and not from map
            auto& handle = make_handle(scorep_metric.name, it.first.name,
                                       it.first.type, socket_num, it.first.quantity);
            result.push_back(scorep_metric);
            requested_sensors_set.insert(handle);
        }
    }

    logging::info() << "added " << result.size() << " sensors";
    metric_properties_added = true;

    // get first data as later ref
    //  hold data read from sysfs file, ordered by socket
    std::shared_ptr<void> raw_occ_buffer(new int8_t[OCC_SENSOR_DATA_BLOCK_SIZE * socket_count]);
    if (nullptr == raw_occ_buffer) {
        fatal("couldn't allocate buffer for occ sensor data");
    }
    read_file_into_buffer(raw_occ_buffer.get(), socket_count);
    init_measurement_data =
        get_sensor_values(raw_occ_buffer.get(), requested_sensors_set, socket_count);
    init_measurement = std::chrono::steady_clock::now();
    last_measurement_data = init_measurement_data;

    return result;
}

void ibmpowernv_sync_plugin::add_metric(const occ_sensor_t& sensor)
{
    logging::trace() << "add_metric called";
    check_fatal();
    logging::debug() << "adding sensor for recording: " << sensor;
}

template <typename Proxy>
void ibmpowernv_sync_plugin::get_current_value(const occ_sensor_t& sensor, Proxy& p)
{
    logging::trace() << "get_current_value called";
    check_fatal();

    /* legacy PTF compatibility, can't we move to <sync, per_host> instead
     * of strict_sync? likely changes in Q-Learning will be needed.
     */
    pid_t ptid = syscall(SYS_gettid);
    if (!this->is_resposible || (this->responsible_thread != ptid)) {
        p.store((int64_t)0);
        return;
    }

    // hold data read from sysfs file, ordered by socket
    std::shared_ptr<void> raw_occ_buffer(new int8_t[OCC_SENSOR_DATA_BLOCK_SIZE * socket_count]);
    if (nullptr == raw_occ_buffer) {
        fatal("couldn't allocate buffer for occ sensor data");
    }

    if ((std::chrono::steady_clock::now() - current_measurement) > min_measurment_interval) {
        // read occ file
        read_file_into_buffer(raw_occ_buffer.get(), socket_count);

        current_measurement_data = get_sensor_values(
            raw_occ_buffer.get(), requested_sensors_set, socket_count);
        current_measurement = std::chrono::steady_clock::now();
    }

    if (sensor.type == occ_sensor_sample_type::acc_derivative) {
        if (sensor.quantity == "W") {
            auto acc_diff = current_measurement_data[sensor].acc_raw -
                            last_measurement_data[sensor].acc_raw;
            auto sample_diff = current_measurement_data[sensor].update_tag -
                               last_measurement_data[sensor].update_tag;
            double power = acc_diff / sample_diff;

            logging::trace()
                << sensor << " = "
                << " current: " << current_measurement_data[sensor].acc_raw
                << ":" << current_measurement_data[sensor].update_tag
                << " last: " << last_measurement_data[sensor].acc_raw << ":"
                << last_measurement_data[sensor].acc_raw << " power: " << power;

            last_measurement_data[sensor] = current_measurement_data[sensor];

            p.write(power); // avg power since last event for this sensor
        }
        else if (sensor.quantity == "J") {
            auto acc_diff = current_measurement_data[sensor].acc_raw -
                            init_measurement_data[sensor].acc_raw;
            auto sample_diff = current_measurement_data[sensor].update_tag -
                               init_measurement_data[sensor].update_tag;
            std::chrono::duration<double> time_diff = current_measurement - init_measurement;
            double energy = acc_diff / sample_diff * time_diff.count();

            logging::trace()
                << sensor << " = "
                << " current: " << current_measurement_data[sensor].acc_raw
                << ":" << current_measurement_data[sensor].update_tag
                << " time: " << time_diff.count() << "s energy: " << energy;

            p.write(energy); // energy since the beginning
        }
        else {
            throw std::runtime_error("unkonw quantity");
        }
    }
    else {
        switch (sensor.get_scorep_type()) {
        case SCOREP_METRIC_VALUE_DOUBLE:
            logging::trace()
                << sensor << " --> fp64: " << current_measurement_data[sensor].fp64;
            p.write(current_measurement_data[sensor].fp64);
            break;

        case SCOREP_METRIC_VALUE_INT64:
            logging::trace() << sensor << " --> int_signed: "
                             << current_measurement_data[sensor].int_signed;
            p.write(current_measurement_data[sensor].int_signed);
            break;

        case SCOREP_METRIC_VALUE_UINT64:
            logging::trace() << sensor << " --> int_unsigned: "
                             << current_measurement_data[sensor].int_unsigned;
            p.write(current_measurement_data[sensor].int_unsigned);
            break;

        default:
            throw std::runtime_error("unidentified type detected: " +
                                     std::to_string(sensor.get_scorep_type()));
        }
    }
}

void ibmpowernv_sync_plugin::synchronize(bool is_responsible, SCOREP_MetricSynchronizationMode sync_mode)
{
    logging::debug() << "Synchronize " << is_responsible << ", " << sync_mode;
    if (is_responsible) {
        switch (sync_mode) {
        case SCOREP_METRIC_SYNCHRONIZATION_MODE_BEGIN:
            logging::debug() << "SCOREP_METRIC_SYNCHRONIZATION_MODE_BEGIN";

            this->is_resposible = true;
            this->responsible_thread = syscall(SYS_gettid);
            logging::debug() << "got responsible ptid:" << this->responsible_thread;

            break;

        case SCOREP_METRIC_SYNCHRONIZATION_MODE_BEGIN_MPP:
            logging::debug() << "SCOREP_METRIC_SYNCHRONIZATION_MODE_BEGIN_MPP";

#ifdef HAVE_MPI
            std::hash<std::string> mpi_color_hash;
            MPI_Comm node_local_comm;
            int myrank_global;
            int myrank_local_node;

            // TODO
            // we are assuming, that the truncated hash (from size_t to int) of the hostname is unique enough to identify a node
            // we probably need a rework here
            int hash = abs(mpi_color_hash(this->hostname));

            logging::debug() << "node identification hash value: " << hash;

            MPI_Comm_rank(MPI_COMM_WORLD, &myrank_global);
            MPI_Comm_split(MPI_COMM_WORLD, hash, myrank_global, &node_local_comm);
            MPI_Comm_rank(node_local_comm, &myrank_local_node);

            // only one (the first) rank per node is responsible to collect actual measurement data
            if (myrank_local_node == 0) {
                this->is_resposible = true;
                logging::debug() << "got responsible process for host: " << this->hostname;
            }
            else {
                // this rank is not the first of this node
                // -> another rank will record data, this one does not have to
                this->is_resposible = false;
            }

            this->responsible_thread = syscall(SYS_gettid);
            logging::debug() << "got responsible ptid:" << this->responsible_thread;

            MPI_Comm_free(&node_local_comm);
#else
            logging::warn()
                << "You are using the non MPI version of this "
                   "plugin. This might lead to trouble if there is more "
                   "than one MPI rank per node.";
            this->is_resposible = true;
            this->responsible_thread = syscall(SYS_gettid);
            logging::debug() << "got responsible ptid:" << this->responsible_thread;
#endif
            break;

        case SCOREP_METRIC_SYNCHRONIZATION_MODE_END:
            break;

        case SCOREP_METRIC_SYNCHRONIZATION_MODE_MAX:
            break;
        }
    }
}

void ibmpowernv_sync_plugin::read_file_into_buffer(void* buf, int socket_count)
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
        fatal("read from occ_sensors_file finished, but did not read enough");
    }
}

SCOREP_METRIC_PLUGIN_CLASS(ibmpowernv_sync_plugin, "ibmpowernv_sync")
