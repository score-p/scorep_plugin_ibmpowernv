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
#ifndef __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__
#define __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__

#include <map>
#include <string>

#include <scorep/SCOREP_MetricTypes.h>
#include <scorep/plugin/plugin.hpp>

/// describes, which value from a sensor record should be recorded
enum occ_sensor_sample_type {
    sample,
    acc,
    timestamp,
    update_tag,
    acc_raw,
    acc_raw_freq,
    acc_derivative,
};

/// contains all information required to locate a sensor and grab its value
struct occ_sensor_t {
    occ_sensor_t(const std::string& name, const occ_sensor_sample_type type, const size_t socket_num, const std::string& quantity="W") : name(name), type(type), socket_num(socket_num), quantity(quantity)
    {
    }
    occ_sensor_t()
    {
    }

    /// name as used in occ_inband_sensors (e.g. PWRSYS)
    std::string name;
    /// what part of the record to record
    occ_sensor_sample_type type = occ_sensor_sample_type::timestamp;
    /// socket (or "chipid") of the sensor
    size_t socket_num = 0;

    std::string quantity;

    SCOREP_MetricValueType get_scorep_type() const;



    /// metric properties for all supported *global* sensors (only present on first socket)
    static const std::map<occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor_master_only;

    /// metric properties for all supported sensors for *every* chip/socket (will be created once per socket)
    static const std::map<occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor_per_socket;
};
typedef struct occ_sensor_t occ_sensor_t;

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs);
bool operator==(const occ_sensor_t& lhs, const occ_sensor_t& rhs);

template <typename T, typename Policies>
using occ_sensor_policy = scorep::plugin::policy::object_id<occ_sensor_t, T, Policies>;

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__
