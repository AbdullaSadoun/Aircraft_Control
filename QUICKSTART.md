# Quick Start Guide

## Installation

```bash
pip install numpy scipy matplotlib control
```

## Run the Complete Project

```bash
python main.py
```

This will:
1. Define the aircraft model (Cessna 172-type)
2. Analyze stability and handling qualities
3. Design the landing controller
4. Simulate an automatic landing
5. Generate visualizations and reports

## Output Files

- `stability_report.txt` - Detailed stability analysis
- `landing_trajectory.png` - Complete landing visualization
- `stability_modes.png` - Eigenvalue plot
- `performance_summary.png` - Landing metrics

## Run Tests

```bash
python test_system.py
```

Validates all components with 5 comprehensive tests.

## Run Examples

```bash
python example_usage.py
```

Demonstrates 5 different use cases:
1. Basic automatic landing
2. Stability analysis at multiple velocities
3. Landing from different altitudes
4. Controller gain sensitivity
5. Visualization generation

## Expected Results

**Landing Performance:**
- Touchdown Speed: ~18 m/s (above stall)
- Sink Rate: ~1.4 m/s (excellent)
- Landing Success: YES ✓

**Stability Analysis:**
- Short Period: Level 1 (excellent damping)
- Phugoid: Unstable (typical for GA, controlled by autopilot)

## Project Structure

```
main.py              - Main execution
aircraft_model.py    - Aircraft dynamics
stability_analysis.py - Stability analysis
landing_controller.py - Automatic controller
simulation.py        - Flight simulator
visualization.py     - Plotting tools
```

## Documentation

- `README.md` - Full documentation
- `PROJECT_REPORT.md` - Technical report
- This file - Quick start guide

For detailed information, see README.md and PROJECT_REPORT.md
