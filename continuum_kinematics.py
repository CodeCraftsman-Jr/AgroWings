"""
Continuum Robot Kinematics for Tendon-Driven Continuum Robots (TDCR)
Implements constant-curvature model based on Webster & Jones (2010)
3-tendon configuration at 120° spacing
"""

import numpy as np
from scipy.optimize import minimize, least_squares
from typing import Tuple, Optional, List
import warnings

class ContinuumKinematics:
    """
    Constant-curvature kinematics for 3-tendon continuum robot
    Based on Webster, Romano, and Cowan (2009) formulation
    """
    
    def __init__(self, 
                 num_sections: int = 1,
                 section_length: float = 150.0,  # mm
                 backbone_radius: float = 5.0,    # mm (radius from center to tendon)
                 servo_drum_radius: float = 10.0,  # mm
                 tendon_angles: List[float] = None):  # degrees
        """
        Initialize continuum robot kinematics
        
        Args:
            num_sections: Number of continuum sections
            section_length: Length of each section at rest (mm)
            backbone_radius: Distance from backbone center to tendons (mm)
            servo_drum_radius: Radius of servo drum for tendon winding (mm)
            tendon_angles: Angular positions of tendons [deg], default [0, 120, 240]
        """
        self.num_sections = num_sections
        self.L0 = section_length  # Rest length
        self.r = backbone_radius  # Tendon radial offset
        self.drum_radius = servo_drum_radius
        
        # Tendon configuration (3 tendons at 120° spacing)
        if tendon_angles is None:
            self.tendon_angles = np.array([0, 120, 240]) * np.pi / 180.0  # radians
        else:
            self.tendon_angles = np.array(tendon_angles) * np.pi / 180.0
        
        self.num_tendons = len(self.tendon_angles)
        
        # Workspace limits
        self.max_curvature = 0.05  # 1/mm (maximum bending)
        self.max_tendon_displacement = 50.0  # mm (maximum tendon pull)
        
        print(f"Continuum Robot Configuration:")
        print(f"  Sections: {num_sections}")
        print(f"  Section length: {section_length} mm")
        print(f"  Backbone radius: {backbone_radius} mm")
        print(f"  Tendons: {self.num_tendons} at {np.degrees(self.tendon_angles)} deg")
    
    def forward_kinematics(self, tendon_lengths: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute end-effector pose from tendon lengths
        Uses constant curvature assumption
        
        Args:
            tendon_lengths: Array of tendon displacements [Δl1, Δl2, Δl3] (mm)
                          Positive = tendon pulled (shortened)
        
        Returns:
            position: End-effector position [x, y, z] (mm)
            orientation: Rotation matrix (3x3)
        """
        # Map tendon lengths to configuration space (κ, φ, L)
        kappa, phi, L = self._tendon_to_configuration(tendon_lengths)
        
        # Compute transformation matrix
        T = self._arc_to_transformation(kappa, phi, L)
        
        # Extract position and orientation
        position = T[:3, 3]
        orientation = T[:3, :3]
        
        return position, orientation
    
    def inverse_kinematics(self, target_position: np.ndarray, 
                          initial_guess: Optional[np.ndarray] = None,
                          method: str = 'least_squares') -> Optional[np.ndarray]:
        """
        Compute tendon lengths to reach target position
        Uses numerical optimization
        
        Args:
            target_position: Target [x, y, z] position (mm)
            initial_guess: Initial tendon lengths guess (mm)
            method: 'least_squares' or 'minimize'
        
        Returns:
            tendon_lengths: Array of tendon displacements [Δl1, Δl2, Δl3] (mm)
                          Returns None if no solution found
        """
        if initial_guess is None:
            initial_guess = np.zeros(self.num_tendons)
        
        def error_function(tendon_lengths):
            """Position error for optimization"""
            try:
                pos, _ = self.forward_kinematics(tendon_lengths)
                error = np.linalg.norm(pos - target_position)
                return error
            except:
                return 1e6  # Large penalty for invalid configurations
        
        def error_vector(tendon_lengths):
            """Vector error for least squares"""
            try:
                pos, _ = self.forward_kinematics(tendon_lengths)
                return pos - target_position
            except:
                return np.ones(3) * 1e6
        
        # Bounds on tendon displacements
        bounds = [(0, self.max_tendon_displacement) for _ in range(self.num_tendons)]
        
        if method == 'least_squares':
            result = least_squares(
                error_vector,
                initial_guess,
                bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
                max_nfev=200
            )
            
            if result.success:
                return result.x
        else:
            result = minimize(
                error_function,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds
            )
            
            if result.success and result.fun < 1.0:  # Position error < 1mm
                return result.x
        
        # No solution found
        return None
    
    def _tendon_to_configuration(self, tendon_lengths: np.ndarray) -> Tuple[float, float, float]:
        """
        Map tendon lengths to configuration space parameters
        
        Args:
            tendon_lengths: Tendon displacements [Δl1, Δl2, Δl3] (mm)
        
        Returns:
            kappa: Curvature (1/mm)
            phi: Bending plane angle (radians)
            L: Arc length (mm)
        """
        # From Webster et al. (2009):
        # For 3 tendons at 120° spacing:
        
        # Average tendon displacement
        l_avg = np.mean(tendon_lengths)
        
        # Arc length (backbone extension/compression)
        L = self.L0 - l_avg
        
        if L <= 0:
            L = 1e-6  # Avoid division by zero
        
        # Compute bending magnitude and direction
        # Using circular configuration mapping
        cos_terms = np.cos(self.tendon_angles)
        sin_terms = np.sin(self.tendon_angles)
        
        # Bending direction components
        delta_x = np.sum((self.L0 - tendon_lengths) * cos_terms)
        delta_y = np.sum((self.L0 - tendon_lengths) * sin_terms)
        
        # Curvature magnitude
        kappa = np.sqrt(delta_x**2 + delta_y**2) / (self.r * L * self.num_tendons / 2.0)
        
        # Bending plane angle
        phi = np.arctan2(delta_y, delta_x)
        
        # Clamp curvature to valid range
        kappa = np.clip(kappa, 0, self.max_curvature)
        
        return kappa, phi, L
    
    def _configuration_to_tendon(self, kappa: float, phi: float, L: float) -> np.ndarray:
        """
        Map configuration space to tendon lengths (inverse of above)
        
        Args:
            kappa: Curvature (1/mm)
            phi: Bending plane angle (radians)
            L: Arc length (mm)
        
        Returns:
            tendon_lengths: Tendon displacements [Δl1, Δl2, Δl3] (mm)
        """
        tendon_lengths = np.zeros(self.num_tendons)
        
        for i in range(self.num_tendons):
            # Tendon length change from curvature
            # Δli = L0 - L - r * κ * L * cos(θi - φ)
            theta_i = self.tendon_angles[i]
            delta_l = self.r * kappa * L * np.cos(theta_i - phi)
            tendon_lengths[i] = (self.L0 - L) + delta_l
        
        return tendon_lengths
    
    def _arc_to_transformation(self, kappa: float, phi: float, L: float) -> np.ndarray:
        """
        Compute homogeneous transformation matrix for constant curvature arc
        
        Args:
            kappa: Curvature (1/mm)
            phi: Bending plane angle (radians)  
            L: Arc length (mm)
        
        Returns:
            T: 4x4 homogeneous transformation matrix
        """
        T = np.eye(4)
        
        if kappa < 1e-6:
            # Straight section
            T[2, 3] = L  # Translation along z-axis
        else:
            # Curved section
            # Radius of curvature
            rho = 1.0 / kappa
            
            # Angle swept by arc
            theta = kappa * L
            
            # Position in bending plane
            x_local = rho * (1 - np.cos(theta))
            z_local = rho * np.sin(theta)
            
            # Rotate to world frame
            cos_phi = np.cos(phi)
            sin_phi = np.sin(phi)
            
            # Position
            T[0, 3] = x_local * cos_phi
            T[1, 3] = x_local * sin_phi  
            T[2, 3] = z_local
            
            # Orientation (rotation about bending axis)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # Rotation matrix construction
            R_phi = np.array([
                [cos_phi, -sin_phi, 0],
                [sin_phi, cos_phi, 0],
                [0, 0, 1]
            ])
            
            R_theta = np.array([
                [cos_theta, 0, sin_theta],
                [0, 1, 0],
                [-sin_theta, 0, cos_theta]
            ])
            
            T[:3, :3] = R_phi @ R_theta @ R_phi.T
        
        return T
    
    def compute_jacobian(self, tendon_lengths: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix relating tendon velocities to end-effector velocities
        
        Args:
            tendon_lengths: Current tendon displacements (mm)
        
        Returns:
            J: 3x3 Jacobian matrix (position only)
        """
        # Numerical Jacobian using finite differences
        epsilon = 0.01  # Small perturbation
        
        pos_current, _ = self.forward_kinematics(tendon_lengths)
        
        J = np.zeros((3, self.num_tendons))
        
        for i in range(self.num_tendons):
            tendon_perturbed = tendon_lengths.copy()
            tendon_perturbed[i] += epsilon
            
            pos_perturbed, _ = self.forward_kinematics(tendon_perturbed)
            
            J[:, i] = (pos_perturbed - pos_current) / epsilon
        
        return J
    
    def is_in_workspace(self, position: np.ndarray) -> bool:
        """
        Check if position is reachable
        
        Args:
            position: Target position [x, y, z] (mm)
        
        Returns:
            True if position is in workspace
        """
        # Simple cylindrical workspace approximation
        x, y, z = position
        
        # Radial distance
        r_xy = np.sqrt(x**2 + y**2)
        
        # Maximum reach approximation
        max_reach = self.L0 * 0.8  # Conservative estimate
        min_height = self.L0 * 0.3
        max_height = self.L0
        
        if z < min_height or z > max_height:
            return False
        
        if r_xy > max_reach * 0.5:  # Radial limit
            return False
        
        return True
    
    def tendon_to_servo_angles(self, tendon_lengths: np.ndarray, 
                               servo_offset: List[float] = None) -> np.ndarray:
        """
        Convert tendon displacements to servo angles
        
        Args:
            tendon_lengths: Tendon displacements [Δl1, Δl2, Δl3] (mm)
            servo_offset: Offset angles for each servo (degrees)
        
        Returns:
            servo_angles: Servo angles in degrees [0-180]
        """
        if servo_offset is None:
            servo_offset = [90, 90, 90]  # Neutral position
        
        servo_angles = np.zeros(self.num_tendons)
        
        for i in range(self.num_tendons):
            # Convert tendon displacement to drum rotation
            # Arc length = radius * angle
            # tendon_length = drum_radius * angle_radians
            angle_rad = tendon_lengths[i] / self.drum_radius
            angle_deg = np.degrees(angle_rad)
            
            # Apply to servo (assume pulling rotates from neutral)
            servo_angles[i] = servo_offset[i] + angle_deg
            
            # Clamp to servo range [0, 180]
            servo_angles[i] = np.clip(servo_angles[i], 0, 180)
        
        return servo_angles
    
    def servo_angles_to_tendon(self, servo_angles: np.ndarray,
                               servo_offset: List[float] = None) -> np.ndarray:
        """
        Convert servo angles to tendon displacements (inverse of above)
        
        Args:
            servo_angles: Servo angles in degrees [0-180]
            servo_offset: Offset angles for each servo (degrees)
        
        Returns:
            tendon_lengths: Tendon displacements (mm)
        """
        if servo_offset is None:
            servo_offset = [90, 90, 90]
        
        tendon_lengths = np.zeros(self.num_tendons)
        
        for i in range(self.num_tendons):
            # Compute angle delta from neutral
            angle_delta_deg = servo_angles[i] - servo_offset[i]
            angle_delta_rad = np.radians(angle_delta_deg)
            
            # Convert to tendon length
            tendon_lengths[i] = angle_delta_rad * self.drum_radius
        
        return tendon_lengths
    
    def get_workspace_points(self, num_samples: int = 100) -> np.ndarray:
        """
        Generate sample points within the workspace
        Useful for visualization and planning
        
        Args:
            num_samples: Number of random samples to generate
        
        Returns:
            points: Array of reachable positions (num_samples x 3)
        """
        points = []
        attempts = 0
        max_attempts = num_samples * 10
        
        while len(points) < num_samples and attempts < max_attempts:
            attempts += 1
            
            # Random tendon lengths
            tendon_lengths = np.random.uniform(0, self.max_tendon_displacement, self.num_tendons)
            
            try:
                pos, _ = self.forward_kinematics(tendon_lengths)
                
                if self.is_in_workspace(pos):
                    points.append(pos)
            except:
                continue
        
        return np.array(points)


# Example usage and testing
if __name__ == "__main__":
    print("="*60)
    print("Continuum Robot Kinematics Test")
    print("="*60)
    
    # Initialize kinematics
    robot = ContinuumKinematics(
        num_sections=1,
        section_length=150.0,  # 150mm arm
        backbone_radius=5.0,   # 5mm from center to tendon
        servo_drum_radius=10.0  # 10mm drum radius
    )
    
    print("\n" + "="*60)
    print("Test 1: Forward Kinematics")
    print("="*60)
    
    # Test: All tendons neutral (straight configuration)
    tendon_lengths = np.array([0.0, 0.0, 0.0])
    pos, rot = robot.forward_kinematics(tendon_lengths)
    print(f"\nNeutral configuration:")
    print(f"  Tendon lengths: {tendon_lengths}")
    print(f"  End position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")
    
    # Test: Pull first tendon (bend in x-direction)
    tendon_lengths = np.array([20.0, 0.0, 0.0])
    pos, rot = robot.forward_kinematics(tendon_lengths)
    print(f"\nPull tendon 1 (20mm):")
    print(f"  Tendon lengths: {tendon_lengths}")
    print(f"  End position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")
    
    # Test: Symmetric pull (extend/compress)
    tendon_lengths = np.array([10.0, 10.0, 10.0])
    pos, rot = robot.forward_kinematics(tendon_lengths)
    print(f"\nSymmetric pull (10mm each):")
    print(f"  Tendon lengths: {tendon_lengths}")
    print(f"  End position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")
    
    print("\n" + "="*60)
    print("Test 2: Inverse Kinematics")
    print("="*60)
    
    # Test: Reach a target position
    target = np.array([20.0, 10.0, 140.0])
    print(f"\nTarget position: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}] mm")
    
    tendon_solution = robot.inverse_kinematics(target)
    
    if tendon_solution is not None:
        print(f"Solution found!")
        print(f"  Tendon lengths: {tendon_solution}")
        
        # Verify solution
        pos_achieved, _ = robot.forward_kinematics(tendon_solution)
        error = np.linalg.norm(pos_achieved - target)
        print(f"  Achieved position: [{pos_achieved[0]:.2f}, {pos_achieved[1]:.2f}, {pos_achieved[2]:.2f}] mm")
        print(f"  Position error: {error:.3f} mm")
        
        # Convert to servo angles
        servo_angles = robot.tendon_to_servo_angles(tendon_solution)
        print(f"  Servo angles: {servo_angles.astype(int)} degrees")
    else:
        print("  No solution found (target may be unreachable)")
    
    print("\n" + "="*60)
    print("Test 3: Workspace Analysis")
    print("="*60)
    
    # Generate workspace samples
    print("\nGenerating workspace samples...")
    workspace_points = robot.get_workspace_points(num_samples=50)
    
    print(f"Generated {len(workspace_points)} reachable points")
    print(f"Workspace bounds:")
    print(f"  X: [{workspace_points[:, 0].min():.1f}, {workspace_points[:, 0].max():.1f}] mm")
    print(f"  Y: [{workspace_points[:, 1].min():.1f}, {workspace_points[:, 1].max():.1f}] mm")
    print(f"  Z: [{workspace_points[:, 2].min():.1f}, {workspace_points[:, 2].max():.1f}] mm")
    
    print("\n" + "="*60)
    print("Test Complete!")
    print("="*60)
