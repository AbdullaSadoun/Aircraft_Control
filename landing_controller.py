"""
Automatic Landing Controller
Implements a multi-phase landing controller including:
1. Glide slope tracking
2. Flare maneuver
3. Touchdown control
"""

import numpy as np
import control as ct

class LandingController:
    """
    Automatic landing controller with glide slope and flare phases
    """
    
    def __init__(self, aircraft_model):
        self.aircraft = aircraft_model
        
        # Landing parameters
        self.glide_slope_angle = np.deg2rad(-3.0)  # -3 degrees
        self.flare_altitude = 20.0  # meters above ground (start flare earlier)
        self.touchdown_velocity = 25.0  # m/s target touchdown speed
        
        # Controller gains (tuned for Cessna 172-like aircraft)
        # Pitch attitude controller
        self.Kp_theta = 3.0
        self.Ki_theta = 0.3
        self.Kd_theta = 1.5
        
        # Altitude controller
        self.Kp_alt = 0.08
        self.Ki_alt = 0.01
        self.Kd_alt = 0.5
        
        # Velocity controller  
        self.Kp_vel = 0.4
        self.Ki_vel = 0.05
        
        # Integrator states
        self.int_theta = 0.0
        self.int_alt = 0.0
        self.int_vel = 0.0
        
        # Previous errors for derivative
        self.prev_error_theta = 0.0
        self.prev_error_alt = 0.0
        
        # Landing phase state
        self.phase = 'approach'  # approach, glide_slope, flare, touchdown
        
    def reset(self):
        """Reset controller integrators"""
        self.int_theta = 0.0
        self.int_alt = 0.0
        self.int_vel = 0.0
        self.prev_error_theta = 0.0
        self.prev_error_alt = 0.0
        self.phase = 'approach'
    
    def compute_glide_slope_reference(self, horizontal_distance):
        """
        Compute reference altitude for glide slope
        
        Args:
            horizontal_distance: distance to touchdown point (m)
            
        Returns:
            reference_altitude: desired altitude (m)
        """
        ref_altitude = -horizontal_distance * np.tan(self.glide_slope_angle)
        return max(0, ref_altitude)
    
    def compute_flare_trajectory(self, current_altitude, current_velocity):
        """
        Compute flare trajectory using exponential flare law
        
        Args:
            current_altitude: current height above ground (m)
            current_velocity: current airspeed (m/s)
            
        Returns:
            desired_pitch_rate: commanded pitch rate (rad/s)
        """
        # Exponential flare parameters
        tau_flare = 5.0  # time constant for flare (increased for smoother flare)
        h_dot_ref = -current_altitude / tau_flare
        
        # Limit sink rate during flare
        h_dot_ref = max(h_dot_ref, -2.5)  # Max 2.5 m/s sink rate
        
        # Desired flight path angle rate
        gamma_dot = h_dot_ref / max(current_velocity, 20.0)  # Avoid division by very small velocity
        
        # For small angles, pitch rate ≈ flight path angle rate
        return gamma_dot
    
    def update_phase(self, altitude, distance_to_touchdown):
        """Update landing phase based on current state"""
        if altitude <= 0.5:
            self.phase = 'touchdown'
        elif altitude < self.flare_altitude:
            self.phase = 'flare'
        elif altitude < 200 and distance_to_touchdown < 4000:  # Established on approach
            self.phase = 'glide_slope'
        else:
            self.phase = 'approach'
    
    def compute_control(self, state, distance_to_touchdown, dt):
        """
        Compute control commands for landing
        
        Args:
            state: dictionary with current state
                   {velocity, altitude, theta, q, alpha}
            distance_to_touchdown: horizontal distance to runway threshold (m)
            dt: time step (s)
            
        Returns:
            control: dictionary with control commands
                    {elevator, throttle}
        """
        velocity = state['velocity']
        altitude = state['altitude']
        theta = state['theta']
        q = state['q']
        
        # Update landing phase
        self.update_phase(altitude, distance_to_touchdown)
        
        elevator = 0.0
        throttle = 0.0
        
        if self.phase == 'approach':
            # Maintain altitude and approach speed
            ref_velocity = 33.0  # m/s
            ref_altitude = altitude  # Hold current altitude until glide slope intercept
            
            # Simple pitch hold for level flight
            theta_cmd = 0.0
            
            # Pitch attitude control
            error_theta = theta_cmd - theta
            self.int_theta += error_theta * dt
            d_error_theta = (error_theta - self.prev_error_theta) / dt
            
            elevator = (self.Kp_theta * error_theta + 
                       self.Ki_theta * self.int_theta + 
                       self.Kd_theta * d_error_theta)
            
            self.prev_error_theta = error_theta
            
            # Velocity control via throttle
            error_vel = ref_velocity - velocity
            self.int_vel += error_vel * dt
            throttle = self.Kp_vel * error_vel + self.Ki_vel * self.int_vel
            
        elif self.phase == 'glide_slope':
            # Track glide slope
            ref_altitude = self.compute_glide_slope_reference(distance_to_touchdown)
            ref_velocity = 30.0  # m/s on glide slope
            
            # Altitude tracking for glide slope
            error_alt = ref_altitude - altitude
            self.int_alt += error_alt * dt
            self.int_alt = np.clip(self.int_alt, -50, 50)  # Anti-windup
            d_error_alt = (error_alt - self.prev_error_alt) / dt
            
            # Convert altitude error to pitch command
            theta_cmd = (0.12 * error_alt + 
                        0.02 * self.int_alt + 
                        0.4 * d_error_alt)
            theta_cmd = np.clip(theta_cmd, -0.12, 0.08)
            
            self.prev_error_alt = error_alt
            
            # Pitch attitude control
            error_theta = theta_cmd - theta
            self.int_theta += error_theta * dt
            d_error_theta = (error_theta - self.prev_error_theta) / dt
            
            elevator = (self.Kp_theta * error_theta + 
                       self.Ki_theta * self.int_theta + 
                       self.Kd_theta * d_error_theta)
            
            self.prev_error_theta = error_theta
            
            # Velocity control
            error_vel = ref_velocity - velocity
            self.int_vel += error_vel * dt
            throttle = 0.7 * (self.Kp_vel * error_vel + self.Ki_vel * self.int_vel)
            
        elif self.phase == 'flare':
            # Execute flare maneuver - reduce sink rate while maintaining speed
            ref_velocity = 27.0  # m/s - maintain higher speed through flare
            
            # Progressive pitch-up command during flare
            # Increases from entry angle to touchdown attitude
            flare_progress = 1.0 - (altitude / self.flare_altitude)
            theta_cmd = np.deg2rad(-2.0) + np.deg2rad(7.0) * flare_progress
            theta_cmd = np.clip(theta_cmd, np.deg2rad(-5.0), np.deg2rad(8.0))
            
            # Aggressive pitch attitude control for flare
            error_theta = theta_cmd - theta
            self.int_theta += error_theta * dt
            d_error_theta = (error_theta - self.prev_error_theta) / dt
            
            elevator = (4.0 * error_theta + 
                       0.8 * self.int_theta + 
                       2.0 * d_error_theta)
            
            self.prev_error_theta = error_theta
            
            # Moderate throttle during flare to maintain speed
            error_vel = ref_velocity - velocity
            self.int_vel += error_vel * dt
            throttle = 0.5 * (self.Kp_vel * error_vel + self.Ki_vel * self.int_vel)
            
        elif self.phase == 'touchdown':
            # Maintain touchdown attitude and idle thrust
            theta_cmd = np.deg2rad(2.0)  # Slight nose-up for touchdown
            
            error_theta = theta_cmd - theta
            elevator = 2.0 * error_theta
            throttle = 0.0  # Idle
        
        # Limit control surfaces
        elevator = np.clip(elevator, -0.4, 0.4)  # rad
        throttle = np.clip(throttle, 0.0, 1.0)
        
        control = {
            'elevator': elevator,
            'throttle': throttle,
            'phase': self.phase
        }
        
        return control
    
    def design_lqr_controller(self, trim_condition):
        """
        Design LQR controller for pitch/altitude tracking
        (Alternative advanced controller design)
        
        Returns:
            K: LQR gain matrix
        """
        A, B = self.aircraft.get_longitudinal_matrices(trim_condition)
        
        # State weighting matrix Q
        # States: [u, w, q, theta]
        Q = np.diag([1.0, 10.0, 5.0, 50.0])
        
        # Control weighting matrix R
        # Controls: [elevator, thrust]
        R = np.diag([1.0, 0.1])
        
        # Solve LQR
        try:
            K, S, E = ct.lqr(A, B, Q, R)
            return K
        except Exception as e:
            print(f"LQR design failed: {e}")
            return None
