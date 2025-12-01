import json
import math

class GridInterpolator:
    def __init__(self, calibration_file="grid_calibration.json", grid_size=5.0):
        self.grid_size = grid_size
        self.data = {}
        try:
            with open(calibration_file, "r") as f:
                raw_data = json.load(f)
                # Parse keys from "x,y" string to tuple (float, float)
                for key, value in raw_data.items():
                    try:
                        x_str, y_str = key.split(',')
                        x, y = float(x_str), float(y_str)
                        self.data[(x, y)] = tuple(value)
                    except ValueError:
                        print(f"Skipping invalid key in calibration data: {key}")
            print(f"Interpolator loaded {len(self.data)} points.")
        except (OSError, ValueError) as e:
            print(f"Failed to load calibration file: {e}")
            self.data = {}

    def get_angles(self, x: float, y: float) -> 'tuple[float, float] | None':
        """
        Calculates servo angles for position (x, y) using bilinear interpolation.
        Returns None if the point is not within a calibrated grid cell.
        """
        if not self.data:
            return None

        # 1. Identify the bottom-left corner of the grid cell
        # We assume the grid is aligned with 0,0 and spaced by grid_size
        # Using floor to find the lower bound index
        # e.g., x=7.5, grid=5 -> i=1 -> x0=5
        
        # Note: We need to handle floating point precision slightly, but standard floor is usually fine for this scale.
        x0 = math.floor(x / self.grid_size) * self.grid_size
        y0 = math.floor(y / self.grid_size) * self.grid_size
        
        x1 = x0 + self.grid_size
        y1 = y0 + self.grid_size

        # 2. Check if we have all 4 corners
        p00 = (x0, y0)
        p10 = (x1, y0)
        p01 = (x0, y1)
        p11 = (x1, y1)

        if p00 not in self.data or p10 not in self.data or \
           p01 not in self.data or p11 not in self.data:
            # We could implement fallback to nearest neighbor or triangle here, 
            # but for now, let's return None to indicate "out of calibrated range".
            return None

        # 3. Retrieve angles
        # Each point is (theta_shoulder, theta_elbow)
        q00 = self.data[p00]
        q10 = self.data[p10]
        q01 = self.data[p01]
        q11 = self.data[p11]

        # 4. Bilinear Interpolation
        # Normalize coordinates to [0, 1]
        u = (x - x0) / self.grid_size
        v = (y - y0) / self.grid_size

        # Interpolate for Shoulder (index 0)
        # f(x,y) = (1-u)(1-v)f(0,0) + u(1-v)f(1,0) + (1-u)v f(0,1) + uv f(1,1)
        shoulder = (1-u)*(1-v)*q00[0] + u*(1-v)*q10[0] + (1-u)*v*q01[0] + u*v*q11[0]

        # Interpolate for Elbow (index 1)
        elbow = (1-u)*(1-v)*q00[1] + u*(1-v)*q10[1] + (1-u)*v*q01[1] + u*v*q11[1]

        return shoulder, elbow
