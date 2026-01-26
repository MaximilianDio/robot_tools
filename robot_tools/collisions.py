import numpy as np

class EllipsoidCollision:
    def __init__(self, obstacle):
        self.obstacle = obstacle
        self.T = np.array(obstacle["T"])
        self.radii = np.array([obstacle["xradius"], obstacle["yradius"], obstacle["zradius"]])
        self.rotation_inv = np.linalg.inv(self.T[:3, :3])
        
        print(f"Ellipsoid initialized with center: {self.T[:3, 3]}, radii: {self.radii}, rotation:\n{self.T[:3, :3]}")


    def is_in_collision(self, point):
        # Transform the point to the ellipsoid's local frame
        local_point = self.rotation_inv @ (point - self.T[:3, 3])
        # Check if the point is inside the ellipsoid
        value = np.sum((local_point / self.radii) ** 2)
        return value <= 1.0  # Inside or on the surface
    
class CylinderCollision:
    def __init__(self, obstacle):
        self.obstacle = obstacle
        self.T = np.array(obstacle["T"])
        self.radius = obstacle["radius"]
        self.height = obstacle["height"]
        self.rotation_inv = np.linalg.inv(self.T[:3, :3])
        
        print(f"Cylinder initialized with center: {self.T[:3, 3]}, radius: {self.radius}, height: {self.height}, rotation:\n{self.T[:3, :3]}")

    def is_in_collision(self, point):
        # Transform the point to the cylinder's local frame
        local_point = self.rotation_inv @ (point - self.T[:3, 3])
        # Check if the point is inside the cylinder
        radial_dist_sq = local_point[0]**2 + local_point[1]**2
        within_radius = radial_dist_sq <= self.radius**2
        within_height = abs(local_point[2]) <= self.height / 2.0
        return within_radius and within_height

class BoxCollision:
    def __init__(self, obstacle):
        self.obstacle = obstacle
        self.T = np.array(obstacle["T"])
        self.size = np.array([obstacle["xsize"], obstacle["ysize"], obstacle["zsize"]])
        self.rotation_inv = np.linalg.inv(self.T[:3, :3])
        
        print(f"Box initialized with center: {self.T[:3, 3]}, size: {self.size}, rotation:\n{self.T[:3, :3]}")

    def is_in_collision(self, point):
        # Transform the point to the box's local frame
        local_point = self.rotation_inv @ (point - self.T[:3, 3])
        # Check if the point is inside the box
        within_x = abs(local_point[0]) <= self.size[0] / 2.0
        within_y = abs(local_point[1]) <= self.size[1] / 2.0
        within_z = abs(local_point[2]) <= self.size[2] / 2.0
        return within_x and within_y and within_z

def create_collision_objects(obstacles_list):
    obstacles = []
    for obstacle in obstacles_list:
        if obstacle["type"] == "ellipsoid":
            obstacles.append(EllipsoidCollision(obstacle))
        elif obstacle["type"] == "cylinder":
            obstacles.append(CylinderCollision(obstacle))
        elif obstacle["type"] == "box":
            obstacles.append(BoxCollision(obstacle))
        else:
            raise ValueError(f"Unknown obstacle type: {obstacle['type']}")
    
    return obstacles
