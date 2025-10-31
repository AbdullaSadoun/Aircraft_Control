"""
Example Usage of the Automatic Landing Controller System
Demonstrates how to use the flight control system for different scenarios
"""

import numpy as np
from aircraft_model import AircraftModel
from stability_analysis import StabilityAnalyzer
from landing_controller import LandingController
from simulation import FlightSimulator
from visualization import Visualizer

def example_1_basic_landing():
    """
    Example 1: Basic automatic landing from cruise altitude
    """
    print("\n" + "="*70)
    print("EXAMPLE 1: Basic Automatic Landing")
    print("="*70)
    
    # Initialize aircraft and controller
    aircraft = AircraftModel()
    controller = LandingController(aircraft)
    simulator = FlightSimulator(aircraft)
    
    # Set up initial condition: 3 km from runway, 300m altitude
    initial_state = {
        'u': 35.0,       # 35 m/s forward velocity
        'w': 0.0,        # Level flight
        'q': 0.0,        # No pitch rate
        'theta': 0.0,    # Level attitude
        'x': 0.0,        # Starting position
        'h': 300.0,      # 300m altitude
        'x_touchdown': 3000.0  # Runway at 3000m
    }
    
    print(f"Initial conditions:")
    print(f"  Altitude: {initial_state['h']:.1f} m")
    print(f"  Speed: {initial_state['u']:.1f} m/s")
    print(f"  Distance to runway: {initial_state['x_touchdown']:.1f} m")
    
    # Run simulation
    print("\nRunning landing simulation...")
    time_hist, state_hist, control_hist = simulator.simulate_landing(
        controller, initial_state, duration=200.0, dt=0.05
    )
    
    # Evaluate performance
    performance = simulator.evaluate_landing_performance(
        time_hist, state_hist, control_hist
    )
    
    print(f"\nLanding completed in {time_hist[-1]:.1f} seconds")
    print(f"\nPerformance:")
    print(f"  Touchdown speed: {performance['touchdown_speed_ms']:.2f} m/s")
    print(f"  Sink rate: {performance['touchdown_vertical_rate_ms']:.2f} m/s")
    print(f"  Pitch at touchdown: {performance['touchdown_pitch_deg']:.1f}°")
    print(f"  Success: {'YES' if performance['landing_success'] else 'NO'}")

def example_2_stability_analysis():
    """
    Example 2: Perform stability analysis at different flight conditions
    """
    print("\n" + "="*70)
    print("EXAMPLE 2: Stability Analysis at Multiple Velocities")
    print("="*70)
    
    aircraft = AircraftModel()
    analyzer = StabilityAnalyzer(aircraft)
    
    velocities = [25, 30, 35, 40]  # m/s
    
    print("\nAnalyzing stability at different velocities:\n")
    print(f"{'Velocity':>10} {'Short Period ζ':>18} {'Short Period ωn':>20} {'Phugoid ζ':>12}")
    print("-" * 70)
    
    for V in velocities:
        trim = aircraft.get_trim_condition(velocity=V)
        modes = analyzer.analyze_longitudinal_stability(trim)
        
        sp_zeta = modes['short_period']['damping_ratio'] if 'short_period' in modes else 0
        sp_wn = modes['short_period']['natural_frequency'] if 'short_period' in modes else 0
        ph_zeta = modes['phugoid']['damping_ratio'] if 'phugoid' in modes else 0
        
        print(f"{V:>8.0f} m/s {sp_zeta:>18.4f} {sp_wn:>18.4f} rad/s {ph_zeta:>12.4f}")
    
    # Detailed analysis at cruise speed
    print(f"\n\nDetailed analysis at 30 m/s:")
    print("-" * 70)
    trim = aircraft.get_trim_condition(velocity=30.0)
    hq = analyzer.evaluate_handling_qualities(trim)
    
    for mode_name, assessment in hq.items():
        print(f"\n{mode_name.upper()}:")
        for key, value in assessment.items():
            print(f"  {key}: {value}")

def example_3_custom_landing():
    """
    Example 3: Landing from different initial conditions
    """
    print("\n" + "="*70)
    print("EXAMPLE 3: Landing from Lower Altitude")
    print("="*70)
    
    aircraft = AircraftModel()
    controller = LandingController(aircraft)
    simulator = FlightSimulator(aircraft)
    
    # Closer and lower initial condition
    initial_state = {
        'u': 30.0,
        'w': -1.5,       # Slight descent
        'q': 0.0,
        'theta': np.deg2rad(-3.0),  # Already on glide slope
        'x': 2000.0,     # 1 km from runway
        'h': 100.0,      # Lower altitude
        'x_touchdown': 3000.0
    }
    
    print(f"Initial conditions:")
    print(f"  Altitude: {initial_state['h']:.1f} m")
    print(f"  Speed: {initial_state['u']:.1f} m/s")
    print(f"  Distance to runway: {initial_state['x_touchdown'] - initial_state['x']:.1f} m")
    print(f"  Pitch angle: {np.rad2deg(initial_state['theta']):.1f}°")
    
    # Run simulation
    print("\nRunning landing simulation...")
    time_hist, state_hist, control_hist = simulator.simulate_landing(
        controller, initial_state, duration=50.0, dt=0.05
    )
    
    # Evaluate performance
    performance = simulator.evaluate_landing_performance(
        time_hist, state_hist, control_hist
    )
    
    print(f"\nLanding completed in {time_hist[-1]:.1f} seconds")
    print(f"\nPerformance:")
    print(f"  Touchdown speed: {performance['touchdown_speed_ms']:.2f} m/s")
    print(f"  Sink rate: {performance['touchdown_vertical_rate_ms']:.2f} m/s")
    print(f"  Success: {'YES' if performance['landing_success'] else 'NO'}")

def example_4_controller_tuning():
    """
    Example 4: Testing controller with different gain settings
    """
    print("\n" + "="*70)
    print("EXAMPLE 4: Controller Gain Sensitivity")
    print("="*70)
    
    aircraft = AircraftModel()
    simulator = FlightSimulator(aircraft)
    
    # Test different pitch control gains
    Kp_values = [2.0, 3.0, 4.0]
    
    initial_state = {
        'u': 30.0,
        'w': -1.0,
        'q': 0.0,
        'theta': 0.0,
        'x': 2000.0,
        'h': 100.0,
        'x_touchdown': 3000.0
    }
    
    print(f"\nTesting different pitch controller gains:\n")
    print(f"{'Kp':>6} {'Touchdown Speed':>18} {'Sink Rate':>12} {'Success':>10}")
    print("-" * 50)
    
    for Kp in Kp_values:
        controller = LandingController(aircraft)
        controller.Kp_theta = Kp  # Modify gain
        
        # Run simulation
        time_hist, state_hist, control_hist = simulator.simulate_landing(
            controller, initial_state.copy(), duration=50.0, dt=0.05
        )
        
        performance = simulator.evaluate_landing_performance(
            time_hist, state_hist, control_hist
        )
        
        print(f"{Kp:>6.1f} {performance['touchdown_speed_ms']:>16.2f} m/s "
              f"{performance['touchdown_vertical_rate_ms']:>10.2f} m/s "
              f"{'YES' if performance['landing_success'] else 'NO':>10}")

def example_5_visualization():
    """
    Example 5: Generate comprehensive visualizations
    """
    print("\n" + "="*70)
    print("EXAMPLE 5: Generating Visualizations")
    print("="*70)
    
    # Run complete simulation
    aircraft = AircraftModel()
    controller = LandingController(aircraft)
    simulator = FlightSimulator(aircraft)
    visualizer = Visualizer()
    analyzer = StabilityAnalyzer(aircraft)
    
    initial_state = {
        'u': 33.0,
        'w': 0.0,
        'q': 0.0,
        'theta': 0.0,
        'x': 0.0,
        'h': 250.0,
        'x_touchdown': 3000.0
    }
    
    print("\nRunning simulation...")
    time_hist, state_hist, control_hist = simulator.simulate_landing(
        controller, initial_state, duration=150.0, dt=0.05
    )
    
    performance = simulator.evaluate_landing_performance(
        time_hist, state_hist, control_hist
    )
    
    print(f"Simulation completed in {time_hist[-1]:.1f} seconds")
    
    # Generate visualizations
    print("\nGenerating visualizations...")
    
    traj_file = visualizer.plot_landing_trajectory(
        time_hist, state_hist, control_hist, 
        filename='example_trajectory.png'
    )
    
    trim = aircraft.get_trim_condition(velocity=30.0)
    modes = analyzer.analyze_longitudinal_stability(trim)
    
    modes_file = visualizer.plot_stability_modes(
        modes, filename='example_stability.png'
    )
    
    perf_file = visualizer.create_performance_summary(
        performance, filename='example_performance.png'
    )
    
    print(f"\nVisualizations saved:")
    print(f"  - {traj_file}")
    print(f"  - {modes_file}")
    print(f"  - {perf_file}")

def main():
    """Run all examples"""
    print("\n" + "="*70)
    print("AUTOMATIC LANDING CONTROLLER - USAGE EXAMPLES")
    print("="*70)
    
    examples = [
        example_1_basic_landing,
        example_2_stability_analysis,
        example_3_custom_landing,
        example_4_controller_tuning,
        example_5_visualization
    ]
    
    for example in examples:
        try:
            example()
        except Exception as e:
            print(f"\nError in {example.__name__}: {e}")
    
    print("\n" + "="*70)
    print("All examples completed!")
    print("="*70)

if __name__ == "__main__":
    main()
