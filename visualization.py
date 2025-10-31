"""
Visualization Tools
Creates plots and visualizations for analysis and results
"""

import numpy as np
import matplotlib.pyplot as plt

class Visualizer:
    """
    Creates visualizations for flight dynamics and control
    """
    
    def __init__(self):
        self.fig_count = 0
    
    def plot_landing_trajectory(self, time_history, state_history, control_history, 
                                filename='landing_trajectory.png'):
        """
        Plot complete landing trajectory with all phases
        """
        # Extract state components
        u = state_history[:, 0]
        w = state_history[:, 1]
        q = state_history[:, 2]
        theta = state_history[:, 3]
        x = state_history[:, 4]
        h = state_history[:, 5]
        
        V = np.sqrt(u**2 + w**2)
        alpha = np.arctan2(w, u)
        
        # Extract controls
        elevator = [c['elevator'] for c in control_history]
        throttle = [c['throttle'] for c in control_history]
        phase = [c['phase'] for c in control_history]
        
        # Create figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('Automatic Landing Trajectory', fontsize=16, fontweight='bold')
        
        # Plot 1: Altitude vs Range
        ax = axes[0, 0]
        ax.plot(x/1000, h, 'b-', linewidth=2, label='Actual')
        
        # Add ideal glide slope
        x_touchdown = 3.0  # km
        x_glide = np.linspace(0, x_touchdown, 100)
        h_glide = (x_touchdown - x_glide) * 1000 * np.tan(np.deg2rad(3.0))
        ax.plot(x_glide, h_glide, 'r--', linewidth=1.5, label='3° Glide Slope')
        
        ax.set_xlabel('Range (km)', fontsize=11)
        ax.set_ylabel('Altitude (m)', fontsize=11)
        ax.set_title('Flight Path', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Plot 2: Velocity
        ax = axes[0, 1]
        ax.plot(time_history, V, 'b-', linewidth=2)
        ax.axhline(y=25, color='r', linestyle='--', label='Target Touchdown Speed')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Velocity (m/s)', fontsize=11)
        ax.set_title('Airspeed', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Plot 3: Pitch Angle and Rate
        ax = axes[1, 0]
        ax.plot(time_history, np.rad2deg(theta), 'b-', linewidth=2, label='Pitch Angle')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Pitch Angle (deg)', fontsize=11)
        ax.set_title('Pitch Attitude', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        ax2 = ax.twinx()
        ax2.plot(time_history, np.rad2deg(q), 'r-', linewidth=1.5, alpha=0.7, label='Pitch Rate')
        ax2.set_ylabel('Pitch Rate (deg/s)', fontsize=11, color='r')
        ax2.tick_params(axis='y', labelcolor='r')
        ax2.legend(loc='upper right')
        
        # Plot 4: Elevator Control
        ax = axes[1, 1]
        ax.plot(time_history[:-1], np.rad2deg(elevator), 'g-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Elevator Deflection (deg)', fontsize=11)
        ax.set_title('Elevator Command', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Plot 5: Throttle
        ax = axes[2, 0]
        ax.plot(time_history[:-1], np.array(throttle)*100, 'orange', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Throttle (%)', fontsize=11)
        ax.set_title('Throttle Setting', fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_ylim([0, 105])
        
        # Plot 6: Landing Phases
        ax = axes[2, 1]
        phase_map = {'approach': 0, 'glide_slope': 1, 'flare': 2, 'touchdown': 3}
        phase_numeric = [phase_map.get(p, 0) for p in phase]
        ax.plot(time_history[:-1], phase_numeric, 'purple', linewidth=2, marker='o', 
                markersize=3, markevery=50)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Phase', fontsize=11)
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(['Approach', 'Glide Slope', 'Flare', 'Touchdown'])
        ax.set_title('Landing Phase', fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Landing trajectory plot saved to {filename}")
        plt.close()
        
        return filename
    
    def plot_stability_modes(self, modes, filename='stability_modes.png'):
        """
        Visualize stability modes on s-plane
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot eigenvalues
        for mode_name, mode_data in modes.items():
            if mode_name.startswith('mode_'):
                eig = mode_data['eigenvalue']
                
                if mode_data['type'] == 'complex':
                    # Plot complex conjugate pair
                    color = 'blue' if mode_data['stability'] == 'stable' else 'red'
                    ax.plot(np.real(eig), np.imag(eig), 'o', color=color, 
                           markersize=10, label=f"{mode_name}")
                    ax.plot(np.real(eig), -np.imag(eig), 'o', color=color, markersize=10)
                else:
                    # Plot real eigenvalue
                    color = 'blue' if mode_data['stability'] == 'stable' else 'red'
                    ax.plot(np.real(eig), 0, 's', color=color, 
                           markersize=10, label=f"{mode_name}")
        
        # Add stability boundary
        ax.axvline(x=0, color='k', linestyle='--', linewidth=1.5, alpha=0.5)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Add damping ratio lines
        zeta_values = [0.1, 0.3, 0.5, 0.7]
        for zeta in zeta_values:
            if zeta < 1:
                theta = np.arccos(zeta)
                r = np.linspace(0, 5, 100)
                x = -r * np.cos(theta)
                y = r * np.sin(theta)
                ax.plot(x, y, 'g--', alpha=0.3, linewidth=0.8)
                ax.plot(x, -y, 'g--', alpha=0.3, linewidth=0.8)
        
        ax.set_xlabel('Real Part', fontsize=12, fontweight='bold')
        ax.set_ylabel('Imaginary Part', fontsize=12, fontweight='bold')
        ax.set_title('Stability Modes (S-Plane)', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Shade stable region
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.axvspan(xlim[0], 0, alpha=0.1, color='green')
        ax.axvspan(0, xlim[1], alpha=0.1, color='red')
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Stability modes plot saved to {filename}")
        plt.close()
        
        return filename
    
    def create_performance_summary(self, performance, filename='performance_summary.png'):
        """
        Create visual summary of landing performance
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.axis('off')
        
        # Title
        title_text = "LANDING PERFORMANCE SUMMARY"
        ax.text(0.5, 0.95, title_text, ha='center', va='top', 
               fontsize=18, fontweight='bold', transform=ax.transAxes)
        
        # Performance metrics
        y_pos = 0.85
        metrics = [
            f"Touchdown Speed: {performance['touchdown_speed_ms']:.2f} m/s",
            f"Vertical Rate at Touchdown: {performance['touchdown_vertical_rate_ms']:.2f} m/s",
            f"Touchdown Pitch Angle: {performance['touchdown_pitch_deg']:.2f}°",
            f"Mean Glide Slope Error: {performance['mean_glide_slope_error_m']:.2f} m",
            f"Max Glide Slope Error: {performance['max_glide_slope_error_m']:.2f} m",
            f"Landing Success: {'YES' if performance['landing_success'] else 'NO'}"
        ]
        
        for metric in metrics:
            ax.text(0.1, y_pos, metric, ha='left', va='top',
                   fontsize=14, transform=ax.transAxes,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
            y_pos -= 0.12
        
        # Pass/Fail criteria
        criteria_text = "\nAcceptance Criteria:\n" \
                       "• Touchdown speed: 18-35 m/s (above stall)\n" \
                       "• Vertical rate: < 3 m/s (FAA Part 23)\n" \
                       "• Glide slope tracking: < 5 m error"
        
        ax.text(0.1, 0.15, criteria_text, ha='left', va='top',
               fontsize=12, transform=ax.transAxes, style='italic')
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Performance summary saved to {filename}")
        plt.close()
        
        return filename
