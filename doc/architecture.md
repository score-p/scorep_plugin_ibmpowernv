# Architecture
This document describes the general architecture of the plugin.

The high-level data flow is as follows:

1. Copy the OCC inband sensors file into a buffer
2. Extract measurements for the requested sensors
3. Compute Metrics for these sensors and expose them to score-p

## Terms
- A **sensor** is a data source for power consumption provided by the OCC.
  It provides:
  - timestamp (512 MHz-based)
  - current reading
  - accumulator
  - number of samples stored in accumulator
- An **OCC sensor type** is one sensor which can be provided by the OCC.
  There can be multiple sensors of the same OCC sensor type,
  e.g. there are two memory *sensors* (one for each processor), but both are of the `OCC_MEM` *OCC sensor type*.
- A **metric** is exposed to score-p and stored into the trace.
  Typically, each sensor is translated to three metrics:
  - The *direct sample*: current reading as-is (W)
  - The *power from energy*: reading derived from accumulator (W)
  - The *energy*: accumulator since start of measurement (J)

## OCC Sensor Type
The OCC sensor type is selected from an enum and used to select the memory region read from the OCC.

OCC-native sensors (prefixed with `OCC_`) are identified by their string name (e.g. `PWRSYS`, `PWRMEM` etc.).

APSS sensors (prefixed with `APSS_`) are provided by the OCC as `APSSCHx` (where `x` is a number from 0-15).
They are identified by their *function ID*, passed in the OCC header.

> The channel number is entirely ignored.

## Sensor
A sensor is defined by the number of the socket to query and which *OCC sensor type* to extract.

Note that not all possible combinations must exist, e.g. the second socket does not provide a bulk power reading.

Each sensor produces the same data, regardless of type.
This resulting data is stored in a `sensor_readout_t`.

