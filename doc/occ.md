# OCC
This document sketches the interface of the OCC as it is used here.
(The OCC interface has further capabilities, though they are not used and hence not discussed here.)

## See also
- [OCC doc](https://raw.githubusercontent.com/open-power/docs/master/occ/OCC_P9_FW_Interfaces.pdf),
  in particular section 11.3 "OCC Main Memory Sensor Data" (p. 142)
- [example implementation](https://github.com/shilpasri/inband_sensors)

## Interface
The data is dumped into a file `/sys/firmware/opal/exports/occ_inband_sensors`, owned by root with read-only permissions for root only.
This file contains one data block per OCC (i.e. per processor).

Notably some sensors are **only on the first** OCC.

The data block is separated into a header section (describes all sensors) and two data buffers,
from which at least one is available.

## Available Sensors
- global (only on first OCC):
  - bulk system power
  - 16 APSS channels, which contain any number of functions.
    Please refer to the [interface doc](https://raw.githubusercontent.com/open-power/docs/master/occ/OCC_P9_FW_Interfaces.pdf) p. 105 for available functions.
- local (exist for every OCC):
  - memory
  - GPU
  - PROC
  - VDN, VDD
    For details see e.g. [this documentation, DOI: 10.1109/ISSCC.2017.7870255](https://ieeexplore.ieee.org/abstract/document/7870255) fig. 3.1.1

## Available Data
The power sensors use the "full reading" data format.

This contains:

- `timestamp`: 512 MHz counter, no inaccuracy in test
- `sample`: latest value, to be scaled by `scale_factor` (1 for power sensors)
- `accumulator`: sum of samples (plain, no postprocessing applied)
- `update_tag`: number of samples stored in accumulator;
  Note that the `update_tag` slightly diverges from the advertised sampling frequency
