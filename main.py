"""
Main Script for MECH 6091 Project
Automatic Landing Controller Development and Validation

This script demonstrates:
1. Aircraft model definition
2. Stability and handling qualities analysis
3. Automatic landing controller design
4. Simulation and validation
"""

import numpy as np
from aircraft_model import AircraftModel
from stability_analysis import StabilityAnalyzer
from landing_controller import LandingController
from simulation import FlightSimulator
from visualization import Visualizer

def main():
    print("=" * 70)
    print("MECH 6091 PROJECT: AUTOMATIC LANDING CONTROLLER")
    print("=" * 70)
    print()
    
    # =========================================================================
    # PHASE 1: Aircraft Model Definition
    # =========================================================================
    print("Phase 1: Aircraft Model Definition")
    print("-" * 70)
    
    aircraft = AircraftModel()
    print(f"Aircraft: General Aviation (Cessna 172-type)")
    print(f"Mass: {aircraft.mass} kg")
    print(f"Wing Area: {aircraft.wing_area} m²")
    print(f"Wing Span: {aircraft.wing_span} m")
    print()
    
    # Get trim condition for cruise
    trim = aircraft.get_trim_condition(velocity=30.0, altitude=0.0)
    print(f"Trim Condition:")
    print(f"  Velocity: {trim['velocity']:.2f} m/s")
    print(f"  Angle of Attack: {np.rad2deg(trim['alpha']):.2f}°")
    print(f"  Elevator: {np.rad2deg(trim['elevator']):.2f}°")
    print(f"  Thrust Required: {trim['thrust']:.2f} N")
    print()
    
    # =========================================================================
    # PHASE 2: Stability and Handling Qualities Analysis
    # =========================================================================
    print("Phase 2: Stability and Handling Qualities Analysis")
    print("-" * 70)
    
    analyzer = StabilityAnalyzer(aircraft)
    
    # Analyze stability at cruise condition
    modes = analyzer.analyze_longitudinal_stability(trim)
    
    print("\nLongitudinal Stability Modes:")
    if 'short_period' in modes:
        sp = modes['short_period']
        print(f"\nShort Period Mode:")
        print(f"  Natural Frequency: {sp['natural_frequency']:.4f} rad/s")
        print(f"  Damping Ratio: {sp['damping_ratio']:.4f}")
        print(f"  Period: {sp['period']:.2f} s")
        print(f"  Stability: {sp['stability']}")
    
    if 'phugoid' in modes:
        ph = modes['phugoid']
        print(f"\nPhugoid Mode:")
        print(f"  Natural Frequency: {ph['natural_frequency']:.4f} rad/s")
        print(f"  Damping Ratio: {ph['damping_ratio']:.4f}")
        print(f"  Period: {ph['period']:.2f} s")
        print(f"  Stability: {ph['stability']}")
    
    # Evaluate handling qualities
    hq = analyzer.evaluate_handling_qualities(trim)
    print(f"\nHandling Qualities Assessment:")
    
    if 'short_period' in hq:
        sp_hq = hq['short_period']
        print(f"  Short Period: Level {sp_hq['level']} - {'Acceptable' if sp_hq['acceptable'] else 'Unacceptable'}")
    
    if 'phugoid' in hq:
        ph_hq = hq['phugoid']
        print(f"  Phugoid: Level {ph_hq['level']} - {'Acceptable' if ph_hq['acceptable'] else 'Unacceptable'}")
    
    # Generate full stability report
    report = analyzer.generate_stability_report(trim)
    with open('stability_report.txt', 'w') as f:
        f.write(report)
    print(f"\nDetailed stability report saved to: stability_report.txt")
    print()
    
    # =========================================================================
    # PHASE 3: Automatic Landing Controller Design
    # =========================================================================
    print("Phase 3: Automatic Landing Controller Design")
    print("-" * 70)
    
    controller = LandingController(aircraft)
    print(f"Landing Controller Parameters:")
    print(f"  Glide Slope Angle: {np.rad2deg(controller.glide_slope_angle):.1f}°")
    print(f"  Flare Altitude: {controller.flare_altitude:.1f} m")
    print(f"  Target Touchdown Speed: {controller.touchdown_velocity:.1f} m/s")
    print(f"\nController Gains:")
    print(f"  Pitch Control - Kp: {controller.Kp_theta}, Ki: {controller.Ki_theta}, Kd: {controller.Kd_theta}")
    print(f"  Altitude Control - Kp: {controller.Kp_alt}, Ki: {controller.Ki_alt}, Kd: {controller.Kd_alt}")
    print(f"  Velocity Control - Kp: {controller.Kp_vel}, Ki: {controller.Ki_vel}")
    
    # Optional: Design LQR controller
    K = controller.design_lqr_controller(trim)
    if K is not None:
        print(f"\nLQR Gains (Alternative Controller):")
        print(f"  K matrix shape: {K.shape}")
    print()
    
    # =========================================================================
    # PHASE 4: Simulation and Validation
    # =========================================================================
    print("Phase 4: Simulation and Validation")
    print("-" * 70)
    
    simulator = FlightSimulator(aircraft)
    
    # Initial conditions for landing approach
    # Aircraft is 3 km from runway, at 300m altitude, flying at 35 m/s
    initial_state = {
        'u': 35.0,      # m/s forward velocity
        'w': 0.0,       # m/s vertical velocity
        'q': 0.0,       # rad/s pitch rate
        'theta': 0.0,   # rad pitch angle
        'x': 0.0,       # m horizontal position
        'h': 300.0,     # m altitude
        'x_touchdown': 3000.0  # m runway threshold position
    }
    
    print(f"Initial Conditions:")
    print(f"  Position: 0 m horizontal, {initial_state['h']} m altitude")
    print(f"  Velocity: {initial_state['u']} m/s")
    print(f"  Distance to Touchdown: 3000 m")
    print()
    
    print("Running simulation...")
    time_history, state_history, control_history = simulator.simulate_landing(
        controller, initial_state, duration=200.0, dt=0.05
    )
    
    print(f"Simulation completed: {len(time_history)} time steps")
    print(f"Total time: {time_history[-1]:.2f} seconds")
    print()
    
    # Evaluate performance
    performance = simulator.evaluate_landing_performance(
        time_history, state_history, control_history
    )
    
    print("Landing Performance:")
    print("-" * 70)
    print(f"  Touchdown Speed: {performance['touchdown_speed_ms']:.2f} m/s")
    print(f"  Vertical Rate at Touchdown: {performance['touchdown_vertical_rate_ms']:.2f} m/s")
    print(f"  Touchdown Pitch Angle: {performance['touchdown_pitch_deg']:.2f}°")
    print(f"  Mean Glide Slope Error: {performance['mean_glide_slope_error_m']:.2f} m")
    print(f"  Max Glide Slope Error: {performance['max_glide_slope_error_m']:.2f} m")
    print(f"  Landing Success: {'YES ✓' if performance['landing_success'] else 'NO ✗'}")
    print()
    
    # =========================================================================
    # PHASE 5: Visualization
    # =========================================================================
    print("Phase 5: Generating Visualizations")
    print("-" * 70)
    
    visualizer = Visualizer()
    
    # Plot landing trajectory
    traj_file = visualizer.plot_landing_trajectory(
        time_history, state_history, control_history
    )
    
    # Plot stability modes
    modes_file = visualizer.plot_stability_modes(modes)
    
    # Create performance summary
    perf_file = visualizer.create_performance_summary(performance)
    
    print()
    print("=" * 70)
    print("PROJECT COMPLETE")
    print("=" * 70)
    print("\nGenerated Files:")
    print(f"  1. stability_report.txt - Detailed stability analysis")
    print(f"  2. {traj_file} - Landing trajectory visualization")
    print(f"  3. {modes_file} - Stability modes plot")
    print(f"  4. {perf_file} - Performance summary")
    print()
    print("Summary:")
    print(f"  Aircraft model: Defined ✓")
    print(f"  Stability analysis: Complete ✓")
    print(f"  Controller design: Complete ✓")
    print(f"  Simulation: Complete ✓")
    print(f"  Landing success: {'YES ✓' if performance['landing_success'] else 'NO ✗'}")
    print()

if __name__ == "__main__":
    main()
