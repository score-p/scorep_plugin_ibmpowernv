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
#include <occ_sensor_t.hpp>

#include <scorep/SCOREP_MetricTypes.h>

#include <map>
#include <string>
#include <stdexcept>

#define OCC_PLUGIN_ADD_SENSOR_DEFAULT(occ_str, scorep_str, desc, unit) \
    {{occ_str, occ_sensor_sample_type::sample, 0},\
     metric_type_constructable(scorep_str, desc, unit, SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_DOUBLE)},\
    {{occ_str, occ_sensor_sample_type::acc_derivative, 0, "J"},\
     metric_type_constructable(scorep_str "_energy", desc " (energy)", "J", SCOREP_METRIC_MODE_ACCUMULATED_LAST, SCOREP_METRIC_VALUE_DOUBLE)},\
    {{occ_str, occ_sensor_sample_type::acc_derivative, 0},\
     metric_type_constructable(scorep_str "_from_energy", desc " (derived from accumulator)", unit, SCOREP_METRIC_MODE_ABSOLUTE_LAST, SCOREP_METRIC_VALUE_DOUBLE)}

const std::map<occ_sensor_sample_type, std::string> name_by_occ_sensor_sample_type = {
    {sample, "sample"},
    {acc, "acc"},
    {timestamp, "timestamp"},
    {update_tag, "update_tag"},
    {acc_raw, "acc_raw"},
    {acc_raw_freq, "acc_raw_freq"},
    {acc_derivative, "acc_derivative"},
};

/// helper class to allow setting all attributes of scorep::plugin::metric_property from constructor
class metric_type_constructable : public scorep::plugin::metric_property {
public:
    metric_type_constructable(std::string name,
                              std::string description,
                              std::string unit,
                              SCOREP_MetricMode _mode = SCOREP_METRIC_MODE_ABSOLUTE_POINT,
                              SCOREP_MetricValueType _type = SCOREP_METRIC_VALUE_DOUBLE,
                              SCOREP_MetricBase _base = SCOREP_METRIC_BASE_DECIMAL,
                              int64_t _exponent = 0) : scorep::plugin::metric_property(name, description, unit) {
        // note: set here, because parent class' attributes can't be set from initializer lists
        mode = _mode;
        type = _type;
        base = _base;
        exponent = _exponent;
    }
};

// note: these sensors get collected *only for the first socket*
// reference: "OCC Firmware Interface Specificationfor POWER9" <https://raw.githubusercontent.com/open-power/docs/master/occ/OCC_P9_FW_Interfaces.pdf> "11.3.2.2 Power Sensors", p. 149
const std::map<legacy_occ_sensor_t, scorep::plugin::metric_property> legacy_occ_sensor_t::metric_properties_by_sensor_master_only = {
    // to add a sensor + derived-from-acc w/ type double you can use this macro:
    // this adds "occ_power_system" and "occ_power_system_from_energy"
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRSYS", "occ_power_system", "power intake of the entire system", "W"),

    // to manually specify sensors use this structure:
    // OCC-string identifier, how/what should be recorded (sample, frequency, derived from accumulator...)
    // name for metric in trace, description, unit
    //
    // example:
    //{{"PWRSYS", occ_sensor_sample_type::sample, 0},
    // metric_type_constructable(
    //     "occ_power_system",
    //     "power intake of the entire system",
    //     "W", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_DOUBLE)},
    //{{"PWRSYS", occ_sensor_sample_type::acc_derivative, 0},
    // metric_type_constructable(
    //     "occ_power_system_from_energy",
    //     "system power derived from energy",
    //     "W", SCOREP_METRIC_MODE_ABSOLUTE_LAST, SCOREP_METRIC_VALUE_DOUBLE)},

    // other metrics that are usually only required for testing. uncomment and recompile to enable
    //{{"PWRSYS", occ_sensor_sample_type::acc},
    // metric_type_constructable(
    //     "occ_power_system_acc",
    //     "accumulator of consumed energy by entire system",
    //     "J", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
    //{{"PWRSYS", occ_sensor_sample_type::timestamp},
    // metric_type_constructable(
    //     "occ_power_system_timestamp",
    //     "timestamp for occ_power_system sensors (@ 512 MHz)",
    //     "ticks", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
    //{{"PWRSYS", occ_sensor_sample_type::update_tag},
    // metric_type_constructable(
    //     "occ_power_system_update_tag",
    //     "number of samples stored in accumulator",
    //     "samples", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
    //{{"PWRSYS", occ_sensor_sample_type::acc_raw},
    // metric_type_constructable(
    //     "occ_power_system_acc_raw",
    //     "raw (unscaled) value of accumulator",
    //     "unscaled energy", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
    //{{"PWRSYS", occ_sensor_sample_type::acc_raw_freq},
    // metric_type_constructable(
    //     "occ_power_system_acc_raw_freq",
    //     "samples per second recorded to accumulator",
    //     "Hz", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_DOUBLE)},
};

const std::map<legacy_occ_sensor_t, scorep::plugin::metric_property> legacy_occ_sensor_t::metric_properties_by_sensor_per_socket = {
    // works same as above
    // is queried for each socket, the final property is built by appending ".SOCKETNUM" (starting w/ 0)
    // -> this yields "occ_power_gpu.0" and "occ_power_gpu_from_energy.0", "occ_power_gpu.1", "occ_power_gpu_from_energy.1"...
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRGPU", "occ_power_gpu", "power consumption of attached GPU(s)", "W"),
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRPROC", "occ_power_proc", "power consumption for this processor", "W"),
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRVDD", "occ_power_vdd", "power consumption for this processor's vdd", "W"),
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRVDN", "occ_power_vdn", "power consumption for this processor's vdn", "W"),
    OCC_PLUGIN_ADD_SENSOR_DEFAULT("PWRMEM", "occ_power_mem", "power consumption for this processor's memory", "W"),

    // when adding sensors w/o the macro keep in mind that you *have* to provide a socket number for the `legacy_occ_sensor_t` data structure
    // this number will be ignored during processing of this sensor list
};

SCOREP_MetricValueType legacy_occ_sensor_t::get_scorep_type() const {
    for (const auto it : legacy_occ_sensor_t::metric_properties_by_sensor_master_only) {
        if (*this == it.first) {
            return it.second.type;
        }
    }

    for (const auto it : legacy_occ_sensor_t::metric_properties_by_sensor_per_socket) {
        // do not compare socket num
        // note: "name" is sth like "PWRSYS" in this context, not the scorep identifier
        if (it.first.type == type && it.first.name == name ) {
            return it.second.type;
        }
    }

    throw std::runtime_error("can't identify datatype for sensor " + name + " with type " + std::to_string(type));
}

bool operator<(const legacy_occ_sensor_t& lhs, const legacy_occ_sensor_t& rhs)
{
    if (lhs.name != rhs.name) {
        return lhs.name < rhs.name;
    }

    if (lhs.type != rhs.type) {
        return lhs.type < rhs.type;
    }

    if (lhs.quantity != rhs.quantity) {
        return lhs.quantity < rhs.quantity;
    }

    return lhs.socket_num < rhs.socket_num;
}

bool operator==(const legacy_occ_sensor_t& lhs, const legacy_occ_sensor_t& rhs) {
    return lhs.name == rhs.name && lhs.type == rhs.type && lhs.socket_num == rhs.socket_num && lhs.quantity == rhs.quantity;
}
