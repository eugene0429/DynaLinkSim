# DynaLinkSim

**DynaLinkSim** is a C++ simulation framework for multi-link, multi-joint mechanical systems. It supports revolute and prismatic joints, and computes the dynamic behavior of articulated mechanisms using numerical integration and constraint solvers.

## Features

- Configurable linkage systems with arbitrary topology  
- Supports revolute and prismatic joints  
- Numerical integration over time  
- Exports simulation results to CSV (position, velocity, constraint errors)

## Build Instructions

This project uses **CMake** for build configuration:

```bash
mkdir build
cd build
cmake ..
make
