from robot_visualization import Robot
import pyvista as pv
import numpy as np
import networkx as nx

def plot_graph(plotter, robot, graph, ee_link_name="CS_6"):
    
    poses = {}
    for key, data in graph.nodes(data=True):
        q = np.fromstring(data['coords'], sep=',')
        pose = robot.fk(q, ee_link_name=ee_link_name)
        poses[key] = pose[:3, 3]
        
    for u, v in graph.edges():
        p1 = poses[u]
        p2 = poses[v]
        line = pv.Line(p1, p2)
        plotter.add_mesh(line, color='gray', line_width=2, opacity=0.5)

if __name__ == "__main__":
    
    plotter = pv.Plotter()  
    plotter.add_axes()
    plotter.camera_position = 'yz'
    plotter.set_focus([0, 0, 0.6])
    plotter.set_position([5, 0, 0.6])
    robot = Robot("robot_tools/robot_assets/urdf/iiwa7.urdf", 
                  plotter,  
                  p0= np.array([0.0,   -1.0,    0.0]), 
                  R0=np.eye(3),
                  color='lightblue', 
                  opacity=1.0)

    robot.set_robot_mesh( id = 0)
    robot.set_robot_mesh( id = 1)

    q0 = np.array([1.0, 0.5, 0, -1.0, 0, 1.0, 0])
    q1 = np.array([0, 0, 0, 0, 0, 0, 0])

    robot.update(q0, id = 0)  # Example joint configuration
    robot.update(q1, id = 1)  # Example joint configuration
    
    # Interpolate between q0 and q1
    
    num_steps = 20
    q = np.linspace(q0, q1, num_steps)
    for i in range(num_steps):

        robot.plot_ee_frame(q[i], ee_link_name="lbr1_gripper_link_ee")

    robot.plot_ee_path(q, ee_link_name="lbr1_gripper_link_ee")
    
    plot_graph(plotter, robot, nx.read_graphml("robot_tools/tests/robot_graph.graphml"), ee_link_name="lbr1_gripper_link_ee")

    plotter.show()  # Display the plotter with the robot mesh   