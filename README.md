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
```

## After Building
After building, the executable `dspSimMain` will be created.

## Running the Simulation
Execute the following command:

```bash
./dspSimMain
```

## Simulation Output
Simulation output will be saved as:
- `q_results_cpp.csv`: Link positions over time
- `qdot_results_cpp.csv`: Link velocities over time
- `c_results_cpp.csv`: Constraint errors over time

## Project Structure
- `src/` – Source code for solvers and link/joint definitions
- `include/` – Header files for core classes
- `result/` – Output files generated after simulation
- `CMakeLists.txt` – CMake build configuration
- `dspSimMain` – Main simulation executable
