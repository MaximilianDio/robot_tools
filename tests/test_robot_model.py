from robot_visualization import Robot
from robot_tools import RobotModel
import pyvista as pv
import numpy as np

if __name__ == "__main__":
    
    plotter = pv.Plotter()  
    plotter.camera_position = 'yz'
    plotter.set_focus([0, 0, 0])
    plotter.set_position([20, 0, 0])
    plotter.add_axes()
    robot = Robot("robot_tools/robot_assets/urdf/simple_robot.urdf", plotter,
                  p0= np.array([0.0,   -2.0,    0.0]), 
                  R0=np.eye(3),
                  color='lightblue',
                  opacity=1.0)
    robot_model = RobotModel("robot_tools/robot_assets/urdf/simple_robot.urdf",
                             p0= np.array([0.0,   -2.0,    0.0]), 
                             R0=np.eye(3))

    robot.set_robot_mesh( id = 0)
    robot.set_robot_mesh( id = 1)

    q0 = np.array([0.0, np.pi/2, 0])
    q1 = np.array([0, -np.pi/2, 0])

    robot.update(q0, id = 0)  # Example joint configuration
    robot.update(q1, id = 1)  # Example joint configuration
    
    # Interpolate between q0 and q1
    
    num_steps = 20
    q = np.linspace(q0, q1, num_steps)
    for i in range(num_steps):

        robot.plot_ee_frame(q[i], ee_link_name="end_effector")
        T,J = robot_model.update_kinematics("end_effector", q[i], np.zeros_like(q[i]))
        
        print(f"step {i}: \nT = {T}, \nJ = {J}")
        
        

    robot.plot_ee_path(q, ee_link_name="end_effector")

    plotter.show()  # Display the plotter with the robot mesh   