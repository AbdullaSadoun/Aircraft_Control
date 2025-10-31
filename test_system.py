"""
Test Script for MECH 6091 Project
Validates all components of the flight control system
"""

import numpy as np
from aircraft_model import AircraftModel
from stability_analysis import StabilityAnalyzer
from landing_controller import LandingController
from simulation import FlightSimulator

def test_aircraft_model():
    """Test aircraft model initialization and trim calculation"""
    print("\n" + "="*70)
    print("TEST 1: Aircraft Model")
    print("="*70)
    
    aircraft = AircraftModel()
    assert aircraft.mass > 0, "Mass should be positive"
    assert aircraft.wing_area > 0, "Wing area should be positive"
    
    trim = aircraft.get_trim_condition(velocity=30.0)
    assert trim['velocity'] == 30.0, "Trim velocity mismatch"
    assert abs(trim['alpha']) < np.pi/4, "Angle of attack should be reasonable"
    
    print("✓ Aircraft model initialized correctly")
    print(f"✓ Trim condition calculated (alpha = {np.rad2deg(trim['alpha']):.2f}°)")
    
    # Test linearization
    A, B = aircraft.get_longitudinal_matrices(trim)
    assert A.shape == (4, 4), "A matrix should be 4x4"
    assert B.shape == (4, 2), "B matrix should be 4x2"
    print("✓ State-space matrices generated correctly")
    
    return True

def test_stability_analysis():
    """Test stability analysis and handling qualities"""
    print("\n" + "="*70)
    print("TEST 2: Stability Analysis")
    print("="*70)
    
    aircraft = AircraftModel()
    analyzer = StabilityAnalyzer(aircraft)
    trim = aircraft.get_trim_condition(velocity=30.0)
    
    modes = analyzer.analyze_longitudinal_stability(trim)
    assert 'short_period' in modes or any('mode_' in k for k in modes.keys()), \
        "Should identify stability modes"
    
    print("✓ Eigenvalue analysis completed")
    
    hq = analyzer.evaluate_handling_qualities(trim)
    assert len(hq) > 0, "Should evaluate handling qualities"
    print("✓ Handling qualities assessed")
    
    report = analyzer.generate_stability_report(trim)
    assert len(report) > 100, "Report should contain substantial content"
    print("✓ Stability report generated")
    
    return True

def test_landing_controller():
    """Test landing controller initialization and control computation"""
    print("\n" + "="*70)
    print("TEST 3: Landing Controller")
    print("="*70)
    
    aircraft = AircraftModel()
    controller = LandingController(aircraft)
    
    assert controller.flare_altitude > 0, "Flare altitude should be positive"
    assert controller.touchdown_velocity > 0, "Touchdown velocity should be positive"
    print("✓ Controller initialized with valid parameters")
    
    # Test control computation
    state = {
        'velocity': 30.0,
        'altitude': 50.0,
        'theta': 0.0,
        'q': 0.0,
        'alpha': 0.1
    }
    
    control = controller.compute_control(state, distance_to_touchdown=1000.0, dt=0.01)
    assert 'elevator' in control, "Control should include elevator"
    assert 'throttle' in control, "Control should include throttle"
    assert 'phase' in control, "Control should indicate landing phase"
    print(f"✓ Control computed (phase: {control['phase']})")
    
    # Test phase transitions
    controller.reset()
    phases_seen = set()
    for alt in [300, 100, 15, 0.1]:
        state['altitude'] = alt
        control = controller.compute_control(state, 500.0, 0.01)
        phases_seen.add(control['phase'])
    
    assert len(phases_seen) >= 2, "Should transition through multiple phases"
    print(f"✓ Phase transitions working (phases: {phases_seen})")
    
    # Test LQR design
    trim = aircraft.get_trim_condition(velocity=30.0)
    K = controller.design_lqr_controller(trim)
    if K is not None:
        assert K.shape[0] == 2, "LQR gain should have 2 rows (controls)"
        print("✓ LQR controller designed successfully")
    
    return True

def test_simulation():
    """Test flight simulator"""
    print("\n" + "="*70)
    print("TEST 4: Flight Simulation")
    print("="*70)
    
    aircraft = AircraftModel()
    simulator = FlightSimulator(aircraft)
    
    # Test dynamics computation
    state = [30.0, -1.0, 0.0, 0.0, 0.0, 100.0]
    control = {'elevator': 0.0, 'throttle': 0.5}
    
    derivatives = simulator.longitudinal_dynamics(state, 0.0, control)
    assert len(derivatives) == 6, "Should return 6 state derivatives"
    print("✓ Dynamics equations evaluated")
    
    # Test short simulation
    controller = LandingController(aircraft)
    initial_state = {
        'u': 30.0,
        'w': -1.0,
        'q': 0.0,
        'theta': 0.0,
        'x': 2000.0,
        'h': 100.0,
        'x_touchdown': 3000.0
    }
    
    time_hist, state_hist, control_hist = simulator.simulate_landing(
        controller, initial_state, duration=10.0, dt=0.05
    )
    
    assert len(time_hist) > 0, "Should produce time history"
    assert state_hist.shape[1] == 6, "State history should have 6 components"
    print(f"✓ Simulation completed ({len(time_hist)} steps)")
    
    # Test performance evaluation
    performance = simulator.evaluate_landing_performance(
        time_hist, state_hist, control_hist
    )
    
    assert 'touchdown_speed_ms' in performance, "Should calculate touchdown speed"
    assert 'landing_success' in performance, "Should determine landing success"
    print(f"✓ Performance metrics calculated")
    
    return True

def test_integration():
    """Test complete system integration"""
    print("\n" + "="*70)
    print("TEST 5: System Integration")
    print("="*70)
    
    # Create all components
    aircraft = AircraftModel()
    analyzer = StabilityAnalyzer(aircraft)
    controller = LandingController(aircraft)
    simulator = FlightSimulator(aircraft)
    
    # Analyze stability
    trim = aircraft.get_trim_condition(velocity=30.0)
    modes = analyzer.analyze_longitudinal_stability(trim)
    hq = analyzer.evaluate_handling_qualities(trim)
    
    print("✓ Stability analysis completed")
    
    # Run landing simulation
    initial_state = {
        'u': 33.0,
        'w': -0.5,
        'q': 0.0,
        'theta': np.deg2rad(-1.0),
        'x': 1500.0,
        'h': 150.0,
        'x_touchdown': 3000.0
    }
    
    time_hist, state_hist, control_hist = simulator.simulate_landing(
        controller, initial_state, duration=50.0, dt=0.05
    )
    
    performance = simulator.evaluate_landing_performance(
        time_hist, state_hist, control_hist
    )
    
    print("✓ Landing simulation completed")
    print(f"  - Touchdown speed: {performance['touchdown_speed_ms']:.2f} m/s")
    print(f"  - Vertical rate: {performance['touchdown_vertical_rate_ms']:.2f} m/s")
    print(f"  - Landing success: {performance['landing_success']}")
    
    # Verify reasonable landing
    assert performance['touchdown_speed_ms'] > 15, "Touchdown speed too low (stall)"
    assert performance['touchdown_speed_ms'] < 40, "Touchdown speed too high"
    assert abs(performance['touchdown_vertical_rate_ms']) < 5, "Vertical rate too high"
    
    print("✓ Landing parameters within acceptable ranges")
    
    return True

def run_all_tests():
    """Run all test cases"""
    print("\n" + "="*70)
    print("MECH 6091 PROJECT - SYSTEM VALIDATION")
    print("="*70)
    
    tests = [
        ("Aircraft Model", test_aircraft_model),
        ("Stability Analysis", test_stability_analysis),
        ("Landing Controller", test_landing_controller),
        ("Flight Simulation", test_simulation),
        ("System Integration", test_integration)
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            if test_func():
                passed += 1
            else:
                failed += 1
                print(f"✗ {name} FAILED")
        except Exception as e:
            failed += 1
            print(f"✗ {name} FAILED with exception: {e}")
    
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Passed: {passed}/{len(tests)}")
    print(f"Failed: {failed}/{len(tests)}")
    
    if failed == 0:
        print("\n✓ ALL TESTS PASSED - System validated successfully!")
    else:
        print(f"\n✗ {failed} test(s) failed")
    
    return failed == 0

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
