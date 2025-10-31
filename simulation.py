"""
Flight Simulation Environment
Simulates aircraft dynamics and landing maneuvers
"""

import numpy as np
from scipy.integrate import odeint

class FlightSimulator:
    """
    6-DOF flight dynamics simulator with focus on longitudinal motion
    """
    
    def __init__(self, aircraft_model):
        self.aircraft = aircraft_model
        
    def longitudinal_dynamics(self, state, t, control):
        """
        Longitudinal equations of motion
        State: [u, w, q, theta, x, h]
        u: velocity along body x-axis (m/s)
        w: velocity along body z-axis (m/s) 
        q: pitch rate (rad/s)
        theta: pitch angle (rad)
        x: horizontal position (m)
        h: altitude (m)
        """
        u, w, q, theta, x, h = state
        
        # Total velocity
        V = np.sqrt(u**2 + w**2)
        
        # Angle of attack
        alpha = np.arctan2(w, u)
        
        # Dynamic pressure
        q_bar = 0.5 * self.aircraft.rho * V**2
        
        # Control inputs
        elevator = control['elevator']
        throttle = control['throttle']
        
        # Aerodynamic forces with proper limiting
        CL = self.aircraft.CL0 + self.aircraft.CLalpha * alpha + self.aircraft.CLde * elevator
        CL = np.clip(CL, -1.5, 1.8)  # Realistic CL limits
        
        CD = self.aircraft.CD0 + self.aircraft.CDalpha * alpha**2 + self.aircraft.CDde * abs(elevator)
        CD = max(CD, 0.02)  # Minimum drag
        
        L = q_bar * self.aircraft.wing_area * CL
        D = q_bar * self.aircraft.wing_area * CD
        
        # Thrust
        T = throttle * self.aircraft.max_thrust
        
        # Forces in body axes
        X = T * np.cos(alpha) - D - self.aircraft.mass * self.aircraft.g * np.sin(theta)
        Z = -T * np.sin(alpha) - L + self.aircraft.mass * self.aircraft.g * np.cos(theta)
        
        # Pitching moment
        Cm = (self.aircraft.Cm0 + 
              self.aircraft.Cmalpha * alpha + 
              self.aircraft.Cmq * q * self.aircraft.chord / (2 * V) +
              self.aircraft.Cmde * elevator)
        
        M = q_bar * self.aircraft.wing_area * self.aircraft.chord * Cm
        
        # State derivatives
        u_dot = X / self.aircraft.mass + w * q
        w_dot = Z / self.aircraft.mass - u * q
        q_dot = M / self.aircraft.Iyy
        theta_dot = q
        
        # Position derivatives (inertial frame)
        x_dot = u * np.cos(theta) + w * np.sin(theta)
        h_dot = u * np.sin(theta) - w * np.cos(theta)
        
        return [u_dot, w_dot, q_dot, theta_dot, x_dot, h_dot]
    
    def simulate_landing(self, controller, initial_state, duration, dt=0.01):
        """
        Simulate complete landing sequence
        
        Args:
            controller: LandingController instance
            initial_state: dictionary with initial conditions
            duration: simulation time (s)
            dt: time step (s)
            
        Returns:
            time_history, state_history, control_history
        """
        # Initialize state vector
        state = np.array([
            initial_state['u'],
            initial_state['w'],
            initial_state['q'],
            initial_state['theta'],
            initial_state['x'],
            initial_state['h']
        ])
        
        # Touchdown position (runway threshold)
        x_touchdown = initial_state.get('x_touchdown', 3000.0)
        
        # Storage for history
        time_history = [0]
        state_history = [state.copy()]
        control_history = []
        
        controller.reset()
        
        t = 0
        while t < duration and state[5] > 0:  # Continue until touchdown or time limit
            # Current state dictionary
            u, w, q, theta, x, h = state
            V = np.sqrt(u**2 + w**2)
            alpha = np.arctan2(w, u)
            
            current_state = {
                'velocity': V,
                'altitude': h,
                'theta': theta,
                'q': q,
                'alpha': alpha,
                'x': x
            }
            
            # Distance to touchdown
            distance_to_touchdown = x_touchdown - x
            
            # Compute control
            control = controller.compute_control(current_state, distance_to_touchdown, dt)
            
            # Integrate dynamics using RK4
            k1 = np.array(self.longitudinal_dynamics(state, t, control))
            k2 = np.array(self.longitudinal_dynamics(state + 0.5*dt*k1, t + 0.5*dt, control))
            k3 = np.array(self.longitudinal_dynamics(state + 0.5*dt*k2, t + 0.5*dt, control))
            k4 = np.array(self.longitudinal_dynamics(state + dt*k3, t + dt, control))
            
            state = state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
            
            # Apply reasonable bounds to prevent numerical issues
            state[0] = np.clip(state[0], 18.0, 45.0)  # forward velocity (allow realistic range)
            state[2] = np.clip(state[2], -0.5, 0.5)  # pitch rate
            state[3] = np.clip(state[3], -0.5, 0.5)  # pitch angle (allow more range)
            
            t += dt
            
            # Store history
            time_history.append(t)
            state_history.append(state.copy())
            control_history.append(control.copy())
            
            # Check for ground contact
            if state[5] <= 0:
                state[5] = 0
                break
        
        return np.array(time_history), np.array(state_history), control_history
    
    def evaluate_landing_performance(self, time_history, state_history, control_history):
        """
        Evaluate landing performance metrics
        
        Returns:
            performance_dict with metrics
        """
        # Extract final state
        final_state = state_history[-1]
        u_td = final_state[0]
        w_td = final_state[1]
        V_td = np.sqrt(u_td**2 + w_td**2)
        theta_td = final_state[3]
        
        # Touchdown metrics
        touchdown_speed = V_td
        # Vertical velocity at touchdown (positive down for sink rate)
        touchdown_rate = -w_td  
        touchdown_pitch = np.rad2deg(theta_td)
        
        # Track glide slope error during approach
        glide_slope_errors = []
        for i, state in enumerate(state_history):
            if control_history and i < len(control_history):
                if control_history[i]['phase'] == 'glide_slope':
                    x = state[4]
                    h = state[5]
                    # Ideal glide slope from 3 degrees
                    x_touchdown = 3000.0
                    distance = x_touchdown - x
                    if distance > 0:
                        ref_h = distance * np.tan(np.deg2rad(3.0))
                        error = abs(h - ref_h)
                        glide_slope_errors.append(error)
        
        # Performance assessment
        # Success criteria: reasonable touchdown parameters
        # Vertical rate < 3 m/s (FAA Part 23 limit is ~10 ft/s = 3.05 m/s)
        # Speed between 18-35 m/s (typical GA aircraft stall ~15-17 m/s)
        performance = {
            'touchdown_speed_ms': touchdown_speed,
            'touchdown_vertical_rate_ms': touchdown_rate,
            'touchdown_pitch_deg': touchdown_pitch,
            'mean_glide_slope_error_m': np.mean(glide_slope_errors) if glide_slope_errors else 0,
            'max_glide_slope_error_m': np.max(glide_slope_errors) if glide_slope_errors else 0,
            'landing_success': abs(touchdown_rate) < 3.0 and touchdown_speed > 18 and touchdown_speed < 35
        }
        
        return performance
