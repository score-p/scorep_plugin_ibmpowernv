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
#include <ostream>
#include <set>

#include <scorep/SCOREP_MetricTypes.h>
#include <scorep/plugin/plugin.hpp>

/// describes, which value from a sensor record should be recorded
enum occ_sensor_sample_type {
    sample,
    timestamp,
    update_tag,
    acc_derivative,
    energy,
};

// TODO enum class for occ_sensor_sample_type

/// common sample types to be provided for each sensor
const std::set<occ_sensor_sample_type> occ_sensor_sample_type_common = {
    occ_sensor_sample_type::sample,
    occ_sensor_sample_type::timestamp,
    occ_sensor_sample_type::update_tag,
    occ_sensor_sample_type::acc_derivative,
    occ_sensor_sample_type::energy,
};

/// str names for occ_sensor_sample_type, used for debugging output
extern const std::map<occ_sensor_sample_type, std::string> name_by_occ_sensor_sample_type;

/** type of sensor
 * Required information to locate a sensor within one OCC sensor **block**
 *
 * Must be one of the predefined values from the documentation.
 *
 * Can be one of:
 *
 * - OCC sensor: sensor which is produced by OCC
 * - APSS sensor: sensor which is provided by APSS (read via APSS[0-15] from OCC)
 *
 * Can be used with a socket number to read its value from the occ inband sensors file.
 *
 * see p. 105 & p. 149 of https://raw.githubusercontent.com/open-power/docs/master/occ/OCC_P9_FW_Interfaces.pdf
 */
enum class occ_power_sensor_type_t {
    // sensors once per system
    OCC_SYS,

    // sensors per processor
    OCC_GPU,
    OCC_PROC,
    OCC_VDD,
    OCC_VDN,
    OCC_MEM,

    // sensors provided via APSS, ordered by function ID!
    // -> APSS function id == occ_power_sensor_t::APSS_[name] - occ_power_sensor_t::APSS_NOT_USED
    APSS_NOT_USED,
    APSS_MEM_PROC0,
    APSS_MEM_PROC1,
    APSS_MEM_PROC2,
    APSS_MEM_PROC3,
    APSS_PROC0,
    APSS_PROC1,
    APSS_PROC2,
    APSS_PROC3,
    APSS_PROC0_CACHE_IO_PCIE,
    APSS_PROC1_CACHE_IO_PCIE,
    APSS_PROC2_CACHE_IO_PCIE,
    APSS_PROC3_CACHE_IO_PCIE,
    APSS_IO_A,
    APSS_IO_B,
    APSS_IO_C,
    APSS_FANS_A,
    APSS_FANS_B,
    APSS_STORAGE_A,
    APSS_STORAGE_B,
    APSS_12V_REMOTE_SENSE,
    APSS_GND_REMOTE_SENSE,
    APSS_TOTAL_SYSTEM_POWER,
    APSS_MEMORY_CACHE,
    APSS_PROC0_GPU0,
    APSS_MEM_PROC0_0,
    APSS_MEM_PROC0_1,
    APSS_MEM_PROC0_2,
    APSS_12V_STANDBY,
    APSS_PROC0_GPU1,
    APSS_PROC0_GPU2,
    APSS_PROC1_GPU0,
    APSS_PROC1_GPU1,
    APSS_PROC1_GPU2,
};

/// sensor types available for first OCC only
const std::set<occ_power_sensor_type_t> occ_power_sensor_type_master_only = {
    occ_power_sensor_type_t::OCC_SYS,
};

/// sensor types available on any OCC (socket)
const std::set<occ_power_sensor_type_t> occ_power_sensor_type_all_occs = {
    occ_power_sensor_type_t::OCC_GPU,
    occ_power_sensor_type_t::OCC_PROC,
    occ_power_sensor_type_t::OCC_VDD,
    occ_power_sensor_type_t::OCC_VDN,
    occ_power_sensor_type_t::OCC_MEM,
};

/**
 * information required to locate & query a sensor
 */
struct occ_sensor_t {
    /// type of sensor to be queried
    occ_power_sensor_type_t sensor_type;

    /// OCC (socket) number of this sensor
    unsigned occ_num;

    /// get name as used by OCC ("PWRSYS" etc.)
    std::string get_occ_name() const {
        const std::map<occ_power_sensor_type_t, std::string> occ_name_by_sensor_type = {
            {occ_power_sensor_type_t::OCC_SYS, "PWRSYS"},
            {occ_power_sensor_type_t::OCC_GPU, "PWRGPU"},
            {occ_power_sensor_type_t::OCC_PROC, "PWRPROC"},
            {occ_power_sensor_type_t::OCC_VDD, "PWRVDD"},
            {occ_power_sensor_type_t::OCC_VDN, "PWRVDN"},
            {occ_power_sensor_type_t::OCC_MEM, "PWRMEM"},
        };

        return occ_name_by_sensor_type.at(sensor_type);
    }

    /// retrieve name to be used in metrics
    std::string get_name() const {
        const std::map<occ_power_sensor_type_t, std::string> name_by_sensor_type = {
            {occ_power_sensor_type_t::OCC_SYS, "system_bulk"},
            {occ_power_sensor_type_t::OCC_GPU, "gpu"},
            {occ_power_sensor_type_t::OCC_PROC, "proc"},
            {occ_power_sensor_type_t::OCC_VDD, "proc_vdd"},
            {occ_power_sensor_type_t::OCC_VDN, "proc_vdn"},
            {occ_power_sensor_type_t::OCC_MEM, "mem"},
        };

        return std::string("occ_power_") + name_by_sensor_type.at(sensor_type) + "." + std::to_string(occ_num);
    }

    /// retrieve description to be used in metrics
    std::string get_description() const {
        const std::map<occ_power_sensor_type_t, std::string> desc_by_sensor_type = {
            {occ_power_sensor_type_t::OCC_SYS, "bulk power consumption of system"},
            {occ_power_sensor_type_t::OCC_GPU, "power consumed by gpus (attached to this socket)"},
            {occ_power_sensor_type_t::OCC_PROC, "power consumed by this processor"},
            {occ_power_sensor_type_t::OCC_VDD, "processor core power consumption"},
            {occ_power_sensor_type_t::OCC_VDN, "processor nest power consumption"},
            {occ_power_sensor_type_t::OCC_MEM, "power consumed by memory"},
        };

        // if is present in every socket: mention socket number in description
        if (occ_power_sensor_type_all_occs.find(sensor_type) != occ_power_sensor_type_all_occs.end()) {
            return desc_by_sensor_type.at(sensor_type) + " (socket " + std::to_string(occ_num) + ")";
        }

        return desc_by_sensor_type.at(sensor_type);
    }

};
using occ_sensor_t = struct occ_sensor_t;

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs);
bool operator==(const occ_sensor_t& lhs, const occ_sensor_t& rhs);
bool operator!=(const occ_sensor_t& lhs, const occ_sensor_t& rhs);


/**
 * a metric which can be stored into a trace
 */
struct occ_metric_t {
    /// constructor to set params
    occ_metric_t(occ_sensor_t sensor, occ_sensor_sample_type sample_type) : sensor(sensor), sample_type(sample_type) {
        // nop
    }

    /// (empty) constructor
    occ_metric_t(){}

    /// the queried sensor
    occ_sensor_t sensor;

    /// which value to extract from the queried sensor
    occ_sensor_sample_type sample_type;

    /// retrieve full name of this metric
    std::string get_name() const {
        const std::map<occ_sensor_sample_type, std::string> name_by_sample_type = {
            {occ_sensor_sample_type::sample, "direct_sample"},
            {occ_sensor_sample_type::timestamp, "timestamp"},
            {occ_sensor_sample_type::update_tag, "update_tag"},
            {occ_sensor_sample_type::acc_derivative, "power_from_energy"},
            {occ_sensor_sample_type::energy, "energy"},
        };

        return sensor.get_name() + "." + name_by_sample_type.at(sample_type);
    }

    /// retrieve description of metric
    std::string get_description() const {
        const std::map<occ_sensor_sample_type, std::string> desc_by_sample_type = {
            {occ_sensor_sample_type::sample, "single sample reported by OCC"},
            {occ_sensor_sample_type::timestamp, "OCC-reported timestamp of last update (512 MHz-based)"},
            {occ_sensor_sample_type::update_tag, "number of samples in accumulator"},
            {occ_sensor_sample_type::acc_derivative, "power derived from accumulator"},
            {occ_sensor_sample_type::energy, "total energy consumed since start"},
        };

        return sensor.get_description() + ": " + desc_by_sample_type.at(sample_type);
    }

    /// get unit of this metric (i.e. recorded into trace)
    std::string get_unit() const {
        const std::map<occ_sensor_sample_type, std::string> unit_by_sample_type = {
            {occ_sensor_sample_type::sample, "W"},
            {occ_sensor_sample_type::timestamp, "#"},
            {occ_sensor_sample_type::update_tag, "#"},
            {occ_sensor_sample_type::acc_derivative, "W"},
            {occ_sensor_sample_type::energy, "J"},
        };

        return unit_by_sample_type.at(sample_type);
    }

    scorep::plugin::metric_property get_metric_property() const {
        scorep::plugin::metric_property mp(get_name(), get_description(), get_unit());

        // only override defaults if known
        if (occ_sensor_sample_type::acc_derivative == sample_type) {
            mp.mode = SCOREP_METRIC_MODE_ABSOLUTE_LAST;
        }

        mp.type = SCOREP_METRIC_VALUE_DOUBLE;
        if (occ_sensor_sample_type::timestamp == sample_type ||
            occ_sensor_sample_type::update_tag == sample_type) {
            mp.type = SCOREP_METRIC_VALUE_UINT64;
        }

        return mp;
    };
};
using occ_metric_t = struct occ_metric_t;

bool operator<(const occ_metric_t& lhs, const occ_metric_t& rhs);
bool operator==(const occ_metric_t& lhs, const occ_metric_t& rhs);

/// contains all information required to locate a sensor and grab its value
struct legacy_occ_sensor_t {
    legacy_occ_sensor_t(const std::string& name, const occ_sensor_sample_type type, const size_t socket_num, const std::string& quantity="W") : name(name), type(type), socket_num(socket_num), quantity(quantity)
    {
    }
    legacy_occ_sensor_t()
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
    static const std::map<legacy_occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor_master_only;

    /// metric properties for all supported sensors for *every* chip/socket (will be created once per socket)
    static const std::map<legacy_occ_sensor_t, scorep::plugin::metric_property> metric_properties_by_sensor_per_socket;
};
typedef struct legacy_occ_sensor_t legacy_occ_sensor_t;

bool operator<(const legacy_occ_sensor_t& lhs, const legacy_occ_sensor_t& rhs);
bool operator==(const legacy_occ_sensor_t& lhs, const legacy_occ_sensor_t& rhs);

inline std::ostream& operator<<(std::ostream& os, const legacy_occ_sensor_t& sensor) 
{
    os << sensor.name << ':' << sensor.socket_num << ':' << name_by_occ_sensor_sample_type.at(sensor.type) << ":"
       << sensor.quantity;
    return os;
}

template <typename T, typename Policies>
using occ_sensor_policy = scorep::plugin::policy::object_id<legacy_occ_sensor_t, T, Policies>;

template <typename T, typename Policies>
using occ_metric_policy = scorep::plugin::policy::object_id<occ_metric_t, T, Policies>;

#endif // __SCOREP_IBMPOWERNV_PLUGIN_OCC_SENSOR_T_HPP_INCLUDED__
