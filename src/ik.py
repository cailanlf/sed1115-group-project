def solve_kinematics(
    target_x: float, target_y: float,
    origin_x: float, origin_y: float,
    shoulder_length: float, elbow_length: float,
    ) -> 'tuple[float, float] | None':
    """
    Get a solution of (alpha, beta) in degrees to move the arm to the specified position.
    Returns None if there is no solution.
    """
    from math import sqrt, sin, cos, acos, atan2, degrees

    x = target_x - origin_x
    y = target_y - origin_y
    L1, L2 = shoulder_length, elbow_length

    AC = sqrt(x*x + y*y)

    # if the target is out of reach, return None
    if AC > L1 + L2 or AC < abs(L1 - L2):
        return None

    angle_CAX = atan2(y, x)
    angle_BAC = acos(
        (L1**2 + AC**2 - L2**2) / (2 * L1 * AC)
    )

    alpha = angle_CAX - angle_BAC
    beta = acos(
        (L1**2 + L2**2 - AC**2) / (2 * L1 * L2)
    )

    return degrees(alpha), degrees(beta)
