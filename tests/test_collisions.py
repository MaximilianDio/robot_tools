from robot_tools import EllipsoidCollision
import numpy as np

if __name__ == "__main__":
    obstacle = {
        "type": "ellipsoid",
        "T": [[1, 0, 0, 1],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]],
        "xradius": 1.0,
        "yradius": 1.0,
        "zradius": 1.0
    }
    
    ellipsoid = EllipsoidCollision(obstacle)
    
    test_points = [
        np.array([-0.0, 0, 0]),  # on surface
        np.array([2, 0.0, 0.0]),  # On surface
        np.array([2, 1, 0.0]),  # Outside
        np.array([1.0, 0.0, 0.0])   # Inside
    ]
    
    for point in test_points:
        collision = ellipsoid.is_in_collision(point)
        status = "in collision" if collision else "not in collision"
        print(f"Point {point} is {status} with the ellipsoid.")