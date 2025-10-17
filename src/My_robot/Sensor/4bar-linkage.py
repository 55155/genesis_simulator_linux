import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy import pi, sin, cos, sqrt, arctan2
from typing import Tuple, Optional


class FourBarLinkage:
    """
    Four-bar Linkage Mechanism Class
    
    Parameters
    ----------
    r1 : float
        Ground link length
    r2 : float
        Crank link length
    r3 : float
        Coupler link length
    r4 : float
        Rocker link length
    ground_position : tuple, optional
        Start and end coordinates of ground link ((x_A, y_A), (x_D, y_D))
    """
    
    def __init__(
        self, 
        r1: float, 
        r2: float, 
        r3: float, 
        r4: float,
        ground_position: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None
    ):
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.r4 = r4
        
        # Set ground link position
        if ground_position is None:
            self.x_A, self.y_A = 0, 0
            self.x_D, self.y_D = r1, 0
        else:
            (self.x_A, self.y_A), (self.x_D, self.y_D) = ground_position
        
        # Position data storage variables
        self.theta2_array = None
        self.theta3_array = None
        self.theta4_array = None
        self.x_B = None
        self.y_B = None
        self.x_C = None
        self.y_C = None
        
        # Animation objects
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.animation = None
        self.traj_x = []
        self.traj_y = []
        
        # Check Grashof condition
        self._check_grashof_condition()
    
    def _check_grashof_condition(self) -> None:
        """Check Grashof condition and print mechanism type"""
        links = [self.r1, self.r2, self.r3, self.r4]
        s = min(links)
        l = max(links)
        p_q_sum = sum(links) - s - l
        
        print(f"\n{'='*50}")
        print("Four-Bar Linkage Mechanism Analysis")
        print(f"{'='*50}")
        print(f"Link lengths: r1={self.r1}, r2={self.r2}, r3={self.r3}, r4={self.r4}")
        print(f"S={s}, L={l}, P+Q={p_q_sum:.2f}")
        
        if s + l < p_q_sum:
            print("✓ Grashof Mechanism: At least one link can rotate 360 degrees")
            
            if self.r1 == s:
                print("  Type: Double Crank")
            elif self.r2 == s or self.r4 == s:
                print("  Type: Crank-Rocker")
            else:
                print("  Type: Double Rocker")
        elif s + l > p_q_sum:
            print("✗ Non-Grashof Mechanism: Triple Rocker")
        else:
            print("◯ Change Point: S + L = P + Q")
        print(f"{'='*50}\n")
    
    def calculate_positions(
        self, 
        rot_num: int = 3, 
        increment: float = 0.05
    ) -> None:
        """
        Calculate joint positions of four-bar linkage
        
        Parameters
        ----------
        rot_num : int
            Number of rotations
        increment : float
            Angle increment (radians)
        """
        print("Starting position calculation...")
        
        # Generate crank angle array
        self.theta2_array = np.arange(0, rot_num * 2 * pi, increment)
        self.theta2_array = np.append(self.theta2_array, rot_num * 2 * pi)
        
        n_points = len(self.theta2_array)
        
        # Initialize arrays
        self.x_B = np.zeros(n_points)
        self.y_B = np.zeros(n_points)
        self.x_C = np.zeros(n_points)
        self.y_C = np.zeros(n_points)
        self.theta3_array = np.zeros(n_points)
        self.theta4_array = np.zeros(n_points)
        
        # Calculate positions for each frame
        for i, theta2 in enumerate(self.theta2_array):
            # Joint B position (crank endpoint)
            self.x_B[i] = self.x_A + self.r2 * cos(theta2)
            self.y_B[i] = self.y_A + self.r2 * sin(theta2)
            
            # Calculate theta4 (vector loop equation)
            K1 = self.r1 / self.r2
            K2 = self.r1 / self.r4
            K3 = (self.r2**2 - self.r3**2 + self.r4**2 + self.r1**2) / (2 * self.r2 * self.r4)
            
            A = cos(theta2) - K1 - K2 * cos(theta2) + K3
            B = -2 * sin(theta2)
            C = K1 - (K2 + 1) * cos(theta2) + K3
            
            discriminant = B**2 - 4*A*C
            
            if discriminant >= 0:
                theta4 = 2 * arctan2(-B + sqrt(discriminant), 2*A)
            else:
                theta4 = self.theta4_array[i-1] if i > 0 else 0
            
            self.theta4_array[i] = theta4
            
            # Calculate theta3
            K4 = self.r1 / self.r3
            K5 = (self.r4**2 - self.r1**2 - self.r2**2 - self.r3**2) / (2 * self.r2 * self.r3)
            
            D = cos(theta2) - K1 + K4 * cos(theta2) + K5
            E = -2 * sin(theta2)
            F = K1 + (K4 - 1) * cos(theta2) + K5
            
            discriminant2 = E**2 - 4*D*F
            
            if discriminant2 >= 0:
                theta3 = 2 * arctan2(-E + sqrt(discriminant2), 2*D)
            else:
                theta3 = self.theta3_array[i-1] if i > 0 else 0
            
            self.theta3_array[i] = theta3
            
            # Joint C position (coupler endpoint)
            self.x_C[i] = self.x_B[i] + self.r3 * cos(theta3)
            self.y_C[i] = self.y_B[i] + self.r3 * sin(theta3)
        
        print(f"✓ Calculation complete! (Total {n_points} frames)")
    
    def get_coupler_point(
        self, 
        ratio: float = 0.5, 
        offset: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return coordinates of a specific point on the coupler link
        
        Parameters
        ----------
        ratio : float
            Ratio on coupler link (0: point B, 1: point C)
        offset : float
            Perpendicular offset from the link
            
        Returns
        -------
        x, y : numpy arrays
            x, y coordinates of the point
        """
        x = self.x_B + ratio * (self.x_C - self.x_B)
        y = self.y_B + ratio * (self.y_C - self.y_B)
        
        if offset != 0:
            # Add perpendicular offset
            theta = self.theta3_array
            x += offset * cos(theta + pi/2)
            y += offset * sin(theta + pi/2)
        
        return x, y
    
    def animate(
        self, 
        interval: int = 30, 
        trace_point: str = 'coupler_mid',
        show_full_trajectory: bool = False,
        figsize: Tuple[int, int] = (14, 6)
    ) -> None:
        """
        Create and display four-bar linkage animation
        
        Parameters
        ----------
        interval : int
            Time interval between frames (milliseconds)
        trace_point : str
            Point to trace trajectory ('coupler_mid', 'joint_B', 'joint_C')
        show_full_trajectory : bool
            Whether to show full trajectory in advance (fade effect)
        figsize : tuple
            Figure size
        """
        if self.theta2_array is None:
            raise ValueError("Please run calculate_positions() first!")
        
        # Initialize trajectory data
        self.traj_x = []
        self.traj_y = []
        self.trace_point = trace_point
        self.show_full_trajectory = show_full_trajectory
        
        # Create figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=figsize)
        
        # Left subplot: linkage mechanism
        self._setup_mechanism_plot()
        
        # Right subplot: angle graph
        self._setup_angle_plot()
        
        # Show full trajectory in advance (option)
        if show_full_trajectory:
            self._draw_full_trajectory()
        
        # Create animation
        print("Creating animation...")
        self.animation = animation.FuncAnimation(
            self.fig, 
            self._animate_frame, 
            init_func=self._init_animation,
            frames=len(self.theta2_array), 
            interval=interval, 
            blit=True, 
            repeat=True
        )
        
        plt.tight_layout()
        plt.show()
    
    def _setup_mechanism_plot(self) -> None:
        """Setup mechanism plot"""
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        # Auto axis range setting
        max_reach = self.r2 + self.r3
        x_center = self.r1 / 2
        self.ax1.set_xlim(x_center - max_reach - 1, x_center + max_reach + 1)
        self.ax1.set_ylim(-max_reach - 1, max_reach + 1)
        
        self.ax1.set_xlabel('X [m]', fontsize=12)
        self.ax1.set_ylabel('Y [m]', fontsize=12)
        self.ax1.set_title('Four-Bar Linkage Mechanism', fontsize=14, fontweight='bold')
        
        # Initialize link line
        self.line, = self.ax1.plot(
            [], [], 'o-', lw=4, color='#2b8cbe', 
            markersize=8, markerfacecolor='red'
        )
        
        # Initialize trajectory line
        self.trajectory, = self.ax1.plot(
            [], [], '-', lw=2, color='orange', alpha=0.8
        )
        
        # Show fixed joints
        self.ax1.plot(self.x_A, self.y_A, 'ko', markersize=12, zorder=5)
        self.ax1.plot(self.x_D, self.y_D, 'ko', markersize=12, zorder=5)
        self.ax1.text(
            self.x_A-0.3, self.y_A-0.4, 'A', 
            fontsize=12, fontweight='bold'
        )
        self.ax1.text(
            self.x_D+0.2, self.y_D-0.4, 'D', 
            fontsize=12, fontweight='bold'
        )
    
    def _draw_full_trajectory(self) -> None:
        """Show full trajectory in semi-transparent"""
        # Extract data for one cycle only
        points_per_cycle = int(2 * pi / (self.theta2_array[1] - self.theta2_array[0]))
        
        if self.trace_point == 'coupler_mid':
            x_full = (self.x_B[:points_per_cycle] + self.x_C[:points_per_cycle]) / 2
            y_full = (self.y_B[:points_per_cycle] + self.y_C[:points_per_cycle]) / 2
        elif self.trace_point == 'joint_B':
            x_full = self.x_B[:points_per_cycle]
            y_full = self.y_B[:points_per_cycle]
        elif self.trace_point == 'joint_C':
            x_full = self.x_C[:points_per_cycle]
            y_full = self.y_C[:points_per_cycle]
        else:
            x_full = (self.x_B[:points_per_cycle] + self.x_C[:points_per_cycle]) / 2
            y_full = (self.y_B[:points_per_cycle] + self.y_C[:points_per_cycle]) / 2
        
        self.ax1.plot(x_full, y_full, '--', lw=1, color='gray', alpha=0.3, zorder=1)
    
    def _setup_angle_plot(self) -> None:
        """Setup angle plot"""
        self.ax2.plot(
            np.degrees(self.theta2_array), 
            np.degrees(self.theta3_array), 
            label='Theta3 (Coupler)', lw=2, color='green'
        )
        self.ax2.plot(
            np.degrees(self.theta2_array), 
            np.degrees(self.theta4_array), 
            label='Theta4 (Rocker)', lw=2, color='purple'
        )
        self.ax2.set_xlabel('Theta2 (Crank Angle) [deg]', fontsize=12)
        self.ax2.set_ylabel('Angle [deg]', fontsize=12)
        self.ax2.set_title('Link Angle Variation', fontsize=14, fontweight='bold')
        self.ax2.legend(fontsize=11)
        self.ax2.grid(True, alpha=0.3)
        
        # Current position marker
        self.current_point, = self.ax2.plot(
            [], [], 'ro', markersize=8
        )
    
    def _init_animation(self):
        """Initialize animation"""
        self.traj_x = []
        self.traj_y = []
        self.line.set_data([], [])
        self.trajectory.set_data([], [])
        self.current_point.set_data([], [])
        return self.line, self.trajectory, self.current_point
    
    def _animate_frame(self, frame: int):
        """Update animation frame"""
        # Draw links
        x_points = [self.x_A, self.x_B[frame], self.x_C[frame], self.x_D]
        y_points = [self.y_A, self.y_B[frame], self.y_C[frame], self.y_D]
        self.line.set_data(x_points, y_points)
        
        # Calculate point to trace
        if self.trace_point == 'coupler_mid':
            traj_x = (self.x_B[frame] + self.x_C[frame]) / 2
            traj_y = (self.y_B[frame] + self.y_C[frame]) / 2
        elif self.trace_point == 'joint_B':
            traj_x = self.x_B[frame]
            traj_y = self.y_B[frame]
        elif self.trace_point == 'joint_C':
            traj_x = self.x_C[frame]
            traj_y = self.y_C[frame]
        else:
            traj_x = (self.x_B[frame] + self.x_C[frame]) / 2
            traj_y = (self.y_B[frame] + self.y_C[frame]) / 2
        
        self.traj_x.append(traj_x)
        self.traj_y.append(traj_y)
        
        # Limit trajectory to recent 1.5 cycles
        points_per_cycle = int(2 * pi / (self.theta2_array[1] - self.theta2_array[0]))
        if len(self.traj_x) > points_per_cycle * 1.5:
            self.traj_x = self.traj_x[-int(points_per_cycle * 1.5):]
            self.traj_y = self.traj_y[-int(points_per_cycle * 1.5):]
        
        self.trajectory.set_data(self.traj_x, self.traj_y)
        
        # Current position on angle graph
        self.current_point.set_data(
            [np.degrees(self.theta2_array[frame])], 
            [np.degrees(self.theta3_array[frame])]
        )
        
        return self.line, self.trajectory, self.current_point
    
    def save_animation(
        self, 
        filename: str, 
        fps: int = 30, 
        writer: str = 'pillow'
    ) -> None:
        """
        Save animation to file
        
        Parameters
        ----------
        filename : str
            File name to save (e.g., 'linkage.gif', 'linkage.mp4')
        fps : int
            Frames per second
        writer : str
            Save library ('pillow', 'ffmpeg', 'imagemagick')
        """
        if self.animation is None:
            raise ValueError("Please run animate() first!")
        
        print(f"Saving animation: {filename}")
        self.animation.save(filename, writer=writer, fps=fps)
        print(f"✓ Save complete: {filename}")
    
    def get_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate angular velocities (numerical differentiation)
        
        Returns
        -------
        omega3, omega4 : numpy arrays
            Angular velocities of coupler and rocker
        """
        omega3 = np.gradient(self.theta3_array, self.theta2_array)
        omega4 = np.gradient(self.theta4_array, self.theta2_array)
        return omega3, omega4
    
    def plot_trajectory(
        self, 
        point_ratio: float = 0.5, 
        offset: float = 0.0,
        num_cycles: int = 1
    ) -> None:
        """
        Plot trajectory of a specific point on coupler link
        
        Parameters
        ----------
        point_ratio : float
            Ratio on coupler link
        offset : float
            Perpendicular offset from link
        num_cycles : int
            Number of cycles to display (1: one cycle only, 0: all)
        """
        if self.theta2_array is None:
            raise ValueError("Please run calculate_positions() first!")
        
        x, y = self.get_coupler_point(point_ratio, offset)
        
        # Calculate points per cycle
        points_per_cycle = int(2 * pi / (self.theta2_array[1] - self.theta2_array[0]))
        
        if num_cycles == 0:
            # Display all
            x_plot = x
            y_plot = y
        else:
            # Display specified number of cycles
            end_idx = min(points_per_cycle * num_cycles + 1, len(x))
            x_plot = x[:end_idx]
            y_plot = y[:end_idx]
        
        fig, ax = plt.subplots(figsize=(9, 9))
        
        # Plot trajectory
        ax.plot(x_plot, y_plot, 'b-', linewidth=2.5, 
                label=f'Coupler Curve (ratio={point_ratio}, offset={offset})')
        
        # Show start and end points
        ax.plot(x_plot[0], y_plot[0], 'go', markersize=12, 
                label='Start Point', zorder=5)
        
        if num_cycles == 1 or (num_cycles > 0 and len(x_plot) >= points_per_cycle):
            ax.plot(x_plot[-1], y_plot[-1], 'ro', markersize=12, 
                    label='End Point (1 cycle)', zorder=5)
        
        # Show fixed joints
        ax.plot(self.x_A, self.y_A, 'ks', markersize=14, 
                label='Fixed Joint A', zorder=5)
        ax.plot(self.x_D, self.y_D, 'ks', markersize=14, 
                label='Fixed Joint D', zorder=5)
        
        ax.set_xlabel('X [m]', fontsize=13)
        ax.set_ylabel('Y [m]', fontsize=13)
        ax.set_title(f'Coupler Curve - {num_cycles if num_cycles > 0 else "All"} Cycle(s)', 
                     fontsize=15, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axis('equal')
        ax.legend(fontsize=10, loc='best')
        plt.tight_layout()
        plt.show()
    
    def __repr__(self) -> str:
        """Class representation"""
        return (f"FourBarLinkage(r1={self.r1}, r2={self.r2}, "
                f"r3={self.r3}, r4={self.r4})")


# ========== Usage Example ==========
if __name__ == "__main__":
    # Create four-bar linkage
    linkage = FourBarLinkage(
        r1=4.0,  # Ground link
        r2=1.0,  # Crank
        r3=3.5,  # Coupler
        r4=3.0   # Rocker
    )
    
    # Calculate positions (one cycle only)
    linkage.calculate_positions(rot_num=1, increment=0.05)
    
    # Run animation (show full trajectory in advance)
    linkage.animate(interval=30, trace_point='coupler_mid', show_full_trajectory=True)
    
    # Plot trajectory for one cycle
    linkage.plot_trajectory(point_ratio=0.5, offset=0.0, num_cycles=1)
    
    # Compare various coupler point trajectories
    # linkage.plot_trajectory(point_ratio=0.3, offset=0.5, num_cycles=1)
    
    # Save animation (optional)
    # linkage.save_animation('four_bar_linkage.gif', fps=30)
