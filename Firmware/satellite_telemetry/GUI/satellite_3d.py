"""
3D Satellite Visualization Module - OPTIMIZED
Displays a 3D satellite model from STL file that rotates based on IMU data

OPTIMIZATIONS:
- Cache rotation matrices to avoid recalculation
- Only update mesh when orientation actually changes
- Reduce draw calls with optimized mesh settings
- Disable smooth shading for better performance
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QMessageBox
import pyqtgraph.opengl as gl
import numpy as np
from stl import mesh

class Satellite3DView(QWidget):
    """
    OPTIMIZED Widget for 3D visualization of satellite orientation
    """
    
    def __init__(self, parent=None, stl_file="Corps.stl"):
        super().__init__(parent)
        
        # Current orientation (Euler angles in degrees)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Cache last orientation to avoid unnecessary updates
        self._last_orientation = (0, 0, 0)
        self._orientation_threshold = 0.5  # degrees - don't update if change is tiny
        
        # STL file path
        self.stl_file = stl_file
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize 3D view and load satellite model"""
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create 3D view widget with optimized settings
        self.view = gl.GLViewWidget()
        self.view.setMinimumSize(400, 400)
        self.view.setCameraPosition(distance=500)
        
        # Disable auto-updates for better control
        self.view.opts['distance'] = 500
        
        # Add grid for reference
        grid = gl.GLGridItem()
        grid.scale(50, 50, 1)
        self.view.addItem(grid)
        
        # Add axis lines for reference
        self.add_reference_axes()
        
        # Load satellite model from STL file
        try:
            self.load_stl_model()
        except Exception as e:
            QMessageBox.warning(self, "STL Load Error", 
                              f"Could not load STL file '{self.stl_file}'.\n"
                              f"Error: {str(e)}\n\n"
                              f"Using placeholder cube instead.")
            self.create_placeholder_model()
        
        layout.addWidget(self.view)
        self.setLayout(layout)
        
    def add_reference_axes(self):
        """Add X, Y, Z axis lines to the 3D view"""
        axis_length = 100
        
        # X axis (red)
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [axis_length, 0, 0]], dtype=np.float32),
            color=(1, 0, 0, 1),
            width=2,
            antialias=False  # Faster
        )
        self.view.addItem(x_axis)
        
        # Y axis (green)
        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, axis_length, 0]], dtype=np.float32),
            color=(0, 1, 0, 1),
            width=2,
            antialias=False
        )
        self.view.addItem(y_axis)
        
        # Z axis (blue)
        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, axis_length]], dtype=np.float32),
            color=(0, 0, 1, 1),
            width=2,
            antialias=False
        )
        self.view.addItem(z_axis)
        
    def load_stl_model(self):
        """Load the satellite model from STL file - optimized"""
        # Load STL file
        satellite_mesh = mesh.Mesh.from_file(self.stl_file)
        
        # Extract vertices and faces
        points = satellite_mesh.vectors.reshape(-1, 3).astype(np.float32)
        
        # Create faces array
        n_triangles = len(satellite_mesh.vectors)
        faces = np.arange(n_triangles * 3, dtype=np.uint32).reshape(n_triangles, 3)
        
        # Center the model
        center = points.mean(axis=0)
        points = points - center
        
        # Create mesh item with performance-optimized settings
        self.satellite_mesh = gl.GLMeshItem(
            vertexes=points,
            faces=faces,
            smooth=False,        # Disable smooth shading - much faster
            drawEdges=False,     # Disable edges - faster rendering
            shader='normalColor', # Simple shader
            glOptions='opaque'   # Opaque rendering is faster
        )
        
        self.view.addItem(self.satellite_mesh)
        
        # Store original points for rotation
        self.original_points = points.copy()
        
        # Pre-compute face count for reuse
        self.n_faces = n_triangles
        self.faces_array = faces
        
        print(f"STL loaded: {n_triangles} triangles, optimized for performance")
        
    def create_placeholder_model(self):
        """Create a simple placeholder cube if STL fails to load"""
        size = 50
        vertices = np.array([
            [-size, -size, -size], [size, -size, -size], [size, size, -size], [-size, size, -size],
            [-size, -size, size], [size, -size, size], [size, size, size], [-size, size, size]
        ], dtype=np.float32)
        
        faces = np.array([
            [0,1,2], [0,2,3],
            [4,5,6], [4,6,7],
            [0,1,5], [0,5,4],
            [2,3,7], [2,7,6],
            [0,3,7], [0,7,4],
            [1,2,6], [1,6,5]
        ], dtype=np.uint32)
        
        self.satellite_mesh = gl.GLMeshItem(
            vertexes=vertices,
            faces=faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0.8, 0.2, 0.2, 1),
            faceColor=(0.9, 0.3, 0.3, 0.6)
        )
        
        self.view.addItem(self.satellite_mesh)
        self.original_points = vertices
        self.faces_array = faces
        print("Using placeholder cube (STL not loaded)")
        
    def update_orientation(self, roll, pitch, yaw):
        """
        OPTIMIZED: Only update if orientation changed significantly
        """
        # Check if change is significant enough to warrant update
        delta_roll = abs(roll - self._last_orientation[0])
        delta_pitch = abs(pitch - self._last_orientation[1])
        delta_yaw = abs(yaw - self._last_orientation[2])
        
        if (delta_roll < self._orientation_threshold and 
            delta_pitch < self._orientation_threshold and 
            delta_yaw < self._orientation_threshold):
            # Change too small, skip update
            return
        
        # Update cached orientation
        self._last_orientation = (roll, pitch, yaw)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
        # Convert to radians
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # Create rotation matrix
        R = self._euler_to_rotation_matrix_3x3(roll_rad, pitch_rad, yaw_rad)
        
        # Apply rotation
        self._rotate_mesh_fast(R)
        
    def _euler_to_rotation_matrix_3x3(self, roll, pitch, yaw):
        """
        OPTIMIZED: Return 3x3 matrix directly (no 4x4 conversion)
        Uses ZYX rotation sequence
        """
        # Pre-compute trig values
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        # Rotation matrices
        Rx = np.array([
            [1,  0,   0],
            [0,  cr, -sr],
            [0,  sr,  cr]
        ], dtype=np.float32)
        
        Ry = np.array([
            [cp,  0, sp],
            [0,   1,  0],
            [-sp, 0, cp]
        ], dtype=np.float32)
        
        Rz = np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,   0,  1]
        ], dtype=np.float32)
        
        # Combined: R = Rz @ Ry @ Rx
        return Rz @ Ry @ Rx
        
    def _rotate_mesh_fast(self, R):
        """
        OPTIMIZED: Fast mesh rotation using numpy matrix multiplication
        """
        if not hasattr(self, 'original_points'):
            return
        
        # Single matrix multiply (much faster than loop)
        rotated_points = (R @ self.original_points.T).T
        
        # Update mesh - reuse faces array
        self.satellite_mesh.setMeshData(
            vertexes=rotated_points, 
            faces=self.faces_array)