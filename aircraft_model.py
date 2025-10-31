"""
Aircraft Model Definition
Defines the aircraft parameters and aerodynamic characteristics
Based on a representative General Aviation aircraft (Cessna 172 type)
"""

import numpy as np

class AircraftModel:
    """
    Aircraft model with parameters for a Cessna 172-like aircraft
    Uses standard aviation conventions and SI units
    """
    
    def __init__(self):
        # Mass and geometry
        self.mass = 1100.0  # kg
        self.wing_area = 16.2  # m^2
        self.wing_span = 11.0  # m
        self.chord = 1.5  # m (mean aerodynamic chord)
        
        # Moments of inertia (kg*m^2)
        self.Ixx = 1285.0
        self.Iyy = 1824.0
        self.Izz = 2666.0
        self.Ixz = 0.0
        
        # Aerodynamic derivatives (longitudinal)
        self.CL0 = 0.28      # Lift coefficient at zero angle of attack
        self.CLalpha = 4.58  # Lift curve slope (per radian)
        self.CLq = 3.9       # Lift due to pitch rate
        self.CLde = 0.43     # Lift due to elevator
        
        self.CD0 = 0.031     # Parasite drag
        self.CDalpha = 0.13  # Drag due to angle of attack
        self.CDde = 0.06     # Drag due to elevator
        
        self.Cm0 = 0.04      # Pitching moment at zero alpha
        self.Cmalpha = -0.61 # Pitching moment due to alpha
        self.Cmq = -12.4     # Pitching moment due to pitch rate
        self.Cmde = -1.28    # Pitching moment due to elevator
        
        # Aerodynamic derivatives (lateral-directional)
        self.CYbeta = -0.31  # Side force due to sideslip
        self.CYdr = 0.187    # Side force due to rudder
        
        self.Clbeta = -0.074 # Roll moment due to sideslip
        self.Clp = -0.410    # Roll damping
        self.Clr = 0.107     # Roll due to yaw rate
        self.Clda = 0.134    # Roll due to aileron
        self.Cldr = 0.0107   # Roll due to rudder
        
        self.Cnbeta = 0.071  # Yaw moment due to sideslip
        self.Cnp = -0.0575   # Yaw due to roll rate
        self.Cnr = -0.125    # Yaw damping
        self.Cnda = -0.0011  # Yaw due to aileron
        self.Cndr = -0.0726  # Yaw due to rudder
        
        # Engine parameters
        self.max_thrust = 1200.0  # N
        
        # Environmental
        self.g = 9.81  # m/s^2
        self.rho = 1.225  # kg/m^3 (sea level)
        
    def get_trim_condition(self, velocity=30.0, altitude=0.0):
        """
        Calculate trim condition for steady level flight
        
        Args:
            velocity: airspeed in m/s
            altitude: altitude in m
            
        Returns:
            Dictionary with trim states and controls
        """
        # Calculate required lift coefficient
        q = 0.5 * self.rho * velocity**2
        CL_trim = (self.mass * self.g) / (q * self.wing_area)
        
        # Calculate angle of attack for trim
        alpha_trim = (CL_trim - self.CL0) / self.CLalpha
        
        # Calculate drag and required thrust
        CD_trim = self.CD0 + self.CDalpha * alpha_trim
        thrust_trim = CD_trim * q * self.wing_area
        
        # Calculate elevator for trim (pitch moment = 0)
        de_trim = -(self.Cm0 + self.Cmalpha * alpha_trim) / self.Cmde
        
        trim_state = {
            'velocity': velocity,
            'alpha': alpha_trim,
            'theta': alpha_trim,  # For level flight, theta = alpha
            'q': 0.0,
            'altitude': altitude,
            'CL': CL_trim,
            'CD': CD_trim,
            'elevator': de_trim,
            'thrust': thrust_trim
        }
        
        return trim_state
    
    def get_longitudinal_matrices(self, trim_condition):
        """
        Get linearized longitudinal dynamics matrices A and B
        State vector: [u, w, q, theta]
        Control vector: [delta_e, delta_T]
        
        Returns:
            A, B matrices for state space representation
        """
        V = trim_condition['velocity']
        alpha = trim_condition['alpha']
        
        q_bar = 0.5 * self.rho * V**2
        S = self.wing_area
        c = self.chord
        m = self.mass
        Iy = self.Iyy
        
        # Stability derivatives
        Xu = -(2 * self.CD0 * q_bar * S) / (m * V)
        Xw = (self.CLalpha - 2 * self.CL0) * q_bar * S / (m * V)
        Zu = -(2 * self.CL0 * q_bar * S) / (m * V)
        Zw = -(self.CLalpha + self.CD0) * q_bar * S / (m * V)
        Zq = -V
        Mu = 0
        Mw = self.Cmalpha * q_bar * S * c / Iy
        Mq = self.Cmq * q_bar * S * c**2 / (2 * Iy * V)
        
        # Control derivatives
        Xde = 0
        Xdt = self.max_thrust / m
        Zde = self.CLde * q_bar * S / m
        Zdt = 0
        Mde = self.Cmde * q_bar * S * c / Iy
        Mdt = 0
        
        # A matrix
        A = np.array([
            [Xu, Xw, 0, -self.g * np.cos(alpha)],
            [Zu, Zw, V + Zq, -self.g * np.sin(alpha)],
            [Mu, Mw, Mq, 0],
            [0, 0, 1, 0]
        ])
        
        # B matrix
        B = np.array([
            [Xde, Xdt],
            [Zde, Zdt],
            [Mde, Mdt],
            [0, 0]
        ])
        
        return A, B
