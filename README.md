# MECH 6091 Project: Automatic Landing Controller

## Aircraft Flight Dynamics and Control System

This project implements a comprehensive flight dynamics and automatic landing control system for a General Aviation aircraft (Cessna 172-type). The project integrates concepts of stability analysis, handling qualities assessment, and automatic control design.

## Project Objectives

✅ **Aircraft Selection**: Implemented a representative GA aircraft model with realistic aerodynamic parameters  
✅ **Stability Analysis**: Comprehensive longitudinal stability and handling qualities evaluation  
✅ **Controller Design**: Multi-phase automatic landing controller (approach, glide slope, flare, touchdown)  
✅ **Validation**: Full 6-DOF simulation with performance assessment

## Features

### 1. Aircraft Model (`aircraft_model.py`)
- Complete aerodynamic parameter definition
- Longitudinal and lateral-directional derivatives
- Trim condition calculation
- State-space linearization

### 2. Stability Analysis (`stability_analysis.py`)
- Eigenvalue analysis for longitudinal modes
- Short period and phugoid mode identification
- MIL-STD-1797 handling qualities assessment
- Comprehensive stability reporting

### 3. Landing Controller (`landing_controller.py`)
- **Approach Phase**: Altitude and speed maintenance
- **Glide Slope Phase**: 3° glide slope tracking
- **Flare Phase**: Exponential flare law implementation
- **Touchdown Phase**: Final attitude control
- PID control architecture with gain scheduling
- Optional LQR controller design

### 4. Flight Simulator (`simulation.py`)
- 6-DOF longitudinal dynamics simulation
- High-fidelity aerodynamic modeling
- RK4 integration for accuracy
- Performance metrics calculation

### 5. Visualization (`visualization.py`)
- Landing trajectory plots
- Stability mode visualization (s-plane)
- Performance summary reports
- Multi-phase landing analysis

## Installation

### Requirements
```bash
pip install -r requirements.txt
```

Dependencies:
- numpy >= 1.21.0
- scipy >= 1.7.0
- matplotlib >= 3.4.0
- control >= 0.9.0

## Usage

### Running the Complete Analysis

```bash
python main.py
```

This will execute:
1. Aircraft model initialization
2. Stability and handling qualities analysis
3. Landing controller design
4. Simulation of automatic landing
5. Performance evaluation and visualization

### Expected Output

The program generates:
- **stability_report.txt**: Detailed stability analysis with eigenvalues and handling qualities
- **landing_trajectory.png**: Complete landing trajectory with 6 subplots showing flight path, velocity, pitch, controls, and phases
- **stability_modes.png**: Eigenvalue plot on s-plane
- **performance_summary.png**: Landing performance metrics

### Example Results

Typical landing performance:
- Touchdown speed: ~25-28 m/s
- Vertical rate: < 2.5 m/s (acceptable sink rate)
- Glide slope tracking: < 3m error
- Pitch angle at touchdown: 2-5°

## Project Structure

```
Aircraft_Control/
├── main.py                  # Main execution script
├── aircraft_model.py        # Aircraft parameters and dynamics
├── stability_analysis.py    # Stability and handling qualities
├── landing_controller.py    # Automatic landing controller
├── simulation.py           # Flight dynamics simulator
├── visualization.py        # Plotting and visualization
├── requirements.txt        # Python dependencies
├── README.md              # This file
└── .gitignore            # Git ignore patterns
```

## Technical Details

### Aircraft Specifications
- **Type**: General Aviation (Cessna 172-like)
- **Mass**: 1100 kg
- **Wing Area**: 16.2 m²
- **Wing Span**: 11.0 m

### Controller Architecture
- **Outer Loop**: Altitude and velocity control
- **Inner Loop**: Pitch attitude and rate control
- **Control Strategy**: Cascaded PID with phase-dependent gain scheduling

### Landing Phases
1. **Approach** (h > 100m): Maintain altitude and approach speed
2. **Glide Slope** (100m > h > 15m): Track 3° glide slope
3. **Flare** (h < 15m): Exponential flare maneuver
4. **Touchdown** (h < 0.5m): Final attitude hold

## Validation

The controller is validated against:
- FAA landing performance requirements
- Typical GA aircraft landing parameters
- MIL-STD-1797 handling qualities criteria

### Success Criteria
- ✅ Touchdown speed: 20-35 m/s
- ✅ Vertical rate: < 3 m/s
- ✅ Glide slope tracking: < 5 m error
- ✅ Stable control throughout all phases

## References

1. Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation*
2. MIL-STD-1797A: *Flying Qualities of Piloted Aircraft*
3. Roskam, J. (1995). *Airplane Flight Dynamics and Automatic Flight Controls*
4. FAA Advisory Circular AC 23-8C: *Flight Test Guide for Certification of Part 23 Airplanes*

## Author

MECH 6091 Course Project - Flight Dynamics and Control

## License

Academic/Educational Use
