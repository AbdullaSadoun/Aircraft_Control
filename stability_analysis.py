"""
Stability and Handling Qualities Analysis
Analyzes aircraft stability characteristics and evaluates handling qualities
"""

import numpy as np
from scipy import linalg
import control as ct

class StabilityAnalyzer:
    """
    Analyzes stability and handling qualities of the aircraft
    """
    
    def __init__(self, aircraft_model):
        self.aircraft = aircraft_model
        
    def analyze_longitudinal_stability(self, trim_condition):
        """
        Analyze longitudinal stability modes
        
        Returns:
            Dictionary with eigenvalues and mode characteristics
        """
        A, B = self.aircraft.get_longitudinal_matrices(trim_condition)
        
        # Calculate eigenvalues and eigenvectors
        eigenvalues, eigenvectors = linalg.eig(A)
        
        # Sort eigenvalues by real part
        idx = np.argsort(np.real(eigenvalues))[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        modes = {}
        
        # Identify modes
        for i, eig in enumerate(eigenvalues):
            if np.isreal(eig):
                # Real eigenvalue (convergence/divergence)
                time_const = -1.0 / np.real(eig) if np.real(eig) != 0 else np.inf
                mode_type = 'stable' if np.real(eig) < 0 else 'unstable'
                modes[f'mode_{i+1}'] = {
                    'eigenvalue': eig,
                    'type': 'real',
                    'stability': mode_type,
                    'time_constant': time_const,
                    'damping_ratio': None,
                    'natural_frequency': None,
                    'period': None
                }
            else:
                # Complex eigenvalue (oscillatory)
                if i > 0 and np.isclose(eigenvalues[i-1], np.conj(eig)):
                    continue  # Skip conjugate pair
                    
                omega_n = np.abs(eig)
                zeta = -np.real(eig) / omega_n
                omega_d = np.imag(eig)
                period = 2 * np.pi / np.abs(omega_d) if omega_d != 0 else np.inf
                
                mode_type = 'stable' if np.real(eig) < 0 else 'unstable'
                
                modes[f'mode_{i+1}'] = {
                    'eigenvalue': eig,
                    'type': 'complex',
                    'stability': mode_type,
                    'damping_ratio': zeta,
                    'natural_frequency': omega_n,
                    'damped_frequency': np.abs(omega_d),
                    'period': period
                }
        
        # Identify specific modes (Short Period and Phugoid)
        complex_modes = [(k, v) for k, v in modes.items() if v['type'] == 'complex']
        
        if len(complex_modes) >= 2:
            # Higher frequency = short period, lower frequency = phugoid
            sorted_complex = sorted(complex_modes, key=lambda x: x[1]['natural_frequency'], reverse=True)
            
            short_period = sorted_complex[0][1]
            phugoid = sorted_complex[1][1]
            
            modes['short_period'] = short_period
            modes['phugoid'] = phugoid
        
        return modes
    
    def evaluate_handling_qualities(self, trim_condition):
        """
        Evaluate handling qualities based on MIL-STD-1797
        
        Returns:
            Dictionary with handling quality ratings
        """
        modes = self.analyze_longitudinal_stability(trim_condition)
        
        hq_assessment = {}
        
        # Short Period Mode Requirements
        if 'short_period' in modes:
            sp = modes['short_period']
            
            # MIL-STD-1797 requirements for Category A flight phases
            # Class II aircraft (medium weight, low-to-medium maneuverability)
            zeta_sp = sp['damping_ratio']
            omega_sp = sp['natural_frequency']
            
            # Level 1: 0.35 < zeta < 1.30
            # Level 2: 0.25 < zeta < 2.00
            # Level 3: 0.15 < zeta
            
            if 0.35 <= zeta_sp <= 1.30:
                sp_level = 1
            elif 0.25 <= zeta_sp <= 2.00:
                sp_level = 2
            elif zeta_sp >= 0.15:
                sp_level = 3
            else:
                sp_level = 4  # Unacceptable
            
            hq_assessment['short_period'] = {
                'damping_ratio': zeta_sp,
                'natural_frequency': omega_sp,
                'level': sp_level,
                'acceptable': sp_level <= 3
            }
        
        # Phugoid Mode Requirements
        if 'phugoid' in modes:
            ph = modes['phugoid']
            
            zeta_ph = ph['damping_ratio']
            
            # Level 1: zeta > 0.04
            # Level 2: zeta > 0
            # Level 3: Time to double > 55 seconds
            
            if zeta_ph >= 0.04:
                ph_level = 1
            elif zeta_ph > 0:
                ph_level = 2
            else:
                # Check time to double amplitude
                time_double = np.log(2) / abs(np.real(ph['eigenvalue']))
                ph_level = 3 if time_double > 55 else 4
            
            hq_assessment['phugoid'] = {
                'damping_ratio': zeta_ph,
                'period': ph['period'],
                'level': ph_level,
                'acceptable': ph_level <= 3
            }
        
        return hq_assessment
    
    def generate_stability_report(self, trim_condition):
        """
        Generate comprehensive stability report
        """
        modes = self.analyze_longitudinal_stability(trim_condition)
        hq = self.evaluate_handling_qualities(trim_condition)
        
        report = []
        report.append("=" * 60)
        report.append("STABILITY ANALYSIS REPORT")
        report.append("=" * 60)
        report.append(f"\nTrim Condition:")
        report.append(f"  Velocity: {trim_condition['velocity']:.2f} m/s")
        report.append(f"  Altitude: {trim_condition['altitude']:.2f} m")
        report.append(f"  Angle of Attack: {np.rad2deg(trim_condition['alpha']):.2f} deg")
        
        report.append(f"\n{'Longitudinal Modes':}")
        report.append("-" * 60)
        
        for mode_name, mode_data in modes.items():
            if mode_name.startswith('mode_'):
                report.append(f"\n{mode_name}:")
                report.append(f"  Eigenvalue: {mode_data['eigenvalue']}")
                report.append(f"  Type: {mode_data['type']}")
                report.append(f"  Stability: {mode_data['stability']}")
                
                if mode_data['type'] == 'complex':
                    report.append(f"  Damping Ratio: {mode_data['damping_ratio']:.4f}")
                    report.append(f"  Natural Frequency: {mode_data['natural_frequency']:.4f} rad/s")
                    report.append(f"  Period: {mode_data['period']:.2f} s")
                else:
                    report.append(f"  Time Constant: {mode_data['time_constant']:.2f} s")
        
        report.append(f"\n{'Handling Qualities Assessment':}")
        report.append("-" * 60)
        
        for mode_name, assessment in hq.items():
            report.append(f"\n{mode_name.upper()}:")
            for key, value in assessment.items():
                report.append(f"  {key}: {value}")
        
        return "\n".join(report)
