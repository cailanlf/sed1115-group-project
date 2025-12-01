import json
import os
from kinematics_interpolator import GridInterpolator

# Create a dummy calibration file
dummy_data = {
    "0.0,0.0": [10, 10],
    "5.0,0.0": [20, 10],
    "0.0,5.0": [10, 20],
    "5.0,5.0": [20, 20]
}

filename = "test_grid_calibration.json"
with open(filename, "w") as f:
    json.dump(dummy_data, f)

print("Created dummy calibration file.")

# Initialize Interpolator
interpolator = GridInterpolator(calibration_file=filename)

# Test Case 1: Exact corner
p1 = (0.0, 0.0)
res1 = interpolator.get_angles(*p1)
print(f"Test (0,0): Expected (10, 10), Got {res1}")
assert res1 == (10, 10)

# Test Case 2: Midpoint (2.5, 2.5)
# Bilinear interpolation of a square where x goes 10->20 and y goes 10->20
# Should be (15, 15)
p2 = (2.5, 2.5)
res2 = interpolator.get_angles(*p2)
print(f"Test (2.5, 2.5): Expected (15, 15), Got {res2}")
assert res2 == (15.0, 15.0)

# Test Case 3: Edge midpoint (2.5, 0)
# Should be (15, 10)
p3 = (2.5, 0.0)
res3 = interpolator.get_angles(*p3)
print(f"Test (2.5, 0): Expected (15, 10), Got {res3}")
assert res3 == (15.0, 10.0)

# Test Case 4: Out of bounds
p4 = (10.0, 10.0)
res4 = interpolator.get_angles(*p4)
print(f"Test (10, 10): Expected None, Got {res4}")
assert res4 is None

# Cleanup
os.remove(filename)
print("All tests passed!")
