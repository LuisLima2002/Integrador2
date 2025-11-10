import math

def angle_between_points(p1, p2, p3):
    """
    Calculates the angle (in degrees) formed by three points (p1, p2, p3) at vertex p2.

    Parameters:
        p1, p2, p3: tuples or lists of coordinates (2D or 3D)
    
    Returns:
        float: angle in degrees
    """
    # Convert to vectors
    v1 = [p1[i] - p2[i] for i in range(len(p2))]
    v2 = [p3[i] - p2[i] for i in range(len(p2))]

    # Dot product and magnitudes
    dot = sum(v1[i] * v2[i] for i in range(len(v1)))
    mag1 = math.sqrt(sum(v1[i] ** 2 for i in range(len(v1))))
    mag2 = math.sqrt(sum(v2[i] ** 2 for i in range(len(v2))))

    # Avoid division by zero
    if mag1 == 0 or mag2 == 0:
        raise ValueError("Two points are identical, cannot define an angle.")

    # Compute angle in radians, then convert to degrees
    cos_theta = dot / (mag1 * mag2)
    # Numerical stability fix (avoid floating-point errors outside [-1, 1])
    cos_theta = max(-1, min(1, cos_theta))
    angle_rad = math.acos(cos_theta)
    
    return math.degrees(angle_rad)

print(angle_between_points([2,-2],[4,0],[0,0]))