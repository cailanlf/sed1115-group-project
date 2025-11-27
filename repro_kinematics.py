import math

def solve_kinematics_original(target_x, target_y):
    origin_x, origin_y = 0, 0
    shoulder_length, elbow_length = 155, 155
    x = target_x - origin_x
    y = target_y - origin_y
    L1, L2 = shoulder_length, elbow_length

    AC = math.sqrt((x - -50)**2 + (y - 139.7)**2)
    
    # Original logic
    try:
        angle_CAAbase = math.asin((x + 50) / AC)
    except ValueError:
        return None
        
    return math.degrees(angle_CAAbase)

def solve_kinematics_fixed(target_x, target_y):
    origin_x, origin_y = 0, 0
    shoulder_length, elbow_length = 155, 155
    x = target_x - origin_x
    y = target_y - origin_y
    L1, L2 = shoulder_length, elbow_length

    AC = math.sqrt((x - -50)**2 + (y - 139.7)**2)
    
    # Fixed logic
    angle_CAAbase = math.atan2(x + 50, 139.7 - y)
        
    return math.degrees(angle_CAAbase)

print("--- Testing Y < 139.7 (e.g. 100) ---")
y_low = 100
print(f"Original (100): {solve_kinematics_original(100, y_low):.2f}")
print(f"Fixed    (100): {solve_kinematics_fixed(100, y_low):.2f}")

print("\n--- Testing Y > 139.7 (e.g. 180) ---")
y_high = 180
print(f"Original (180): {solve_kinematics_original(100, y_high):.2f}")
print(f"Fixed    (180): {solve_kinematics_fixed(100, y_high):.2f}")
