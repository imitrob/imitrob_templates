import rclpy
from rclpy.node import Node
from imitrob_robot_client.robot_client import RobotActionClient

def test_execute_points(robot_client):
    for z in [0.3, 0.25, 0.2, 0.15, 0.1]:
        robot_client.move_pose(p=[0.5, 0.0, z], q=[1., 0., 0., 0.])
    
    print("test_execute_points PASSED")
    input("now waiting")

def test_execute_trajectory(robot_client):
    p, q = [], []
    for z in [0.3, 0.25, 0.2, 0.15, 0.1]:
        p.append([0.5, 0.0, z])
        q.append([1., 0., 0., 0.])
    robot_client.trajectory_pose(p, q)

    print("test_execute_trajectory PASSED")
    input("now waiting")

def test_move_gripper(robot_client):

    robot_client.close_gripper()

    print("GRIPPER CLOSED")
    input("now waiting")

    robot_client.open_gripper()

    print("GRIPPER OPENED")
    input("now waiting")


def test_serialization(robot_client):
    for z in [0.4,0.45,0.48]:
        robot_client.open_gripper()
        robot_client.close_gripper()
        robot_client.move_pose(p=[0.3, 0.0, z], q=[1., 0., 0., 0.])
    
    robot_client.open_gripper()
    print("test_execute_points PASSED")

def test_move_home(robot_client):
    for z in [0.5]:
        robot_client.move_pose(p=[0.3, 0.0, z], q=[1., 0., 0., 0.])
    
    print("test_execute_points PASSED")

def init():
    rclpy.init()
    rosnode = Node("test_robot_moving_node")
    robot_client = RobotActionClient(rosnode)
    robot_client.start()
    return robot_client

def main():
    robot_client = init()
    test_execute_points(robot_client)
    test_execute_trajectory(robot_client)
    test_move_gripper(robot_client)
    test_serialization(robot_client)
    test_move_home(robot_client)

def send_move():
    robot_client = init()
    robot_client.move_pose(p=[0.3, 0.0, 0.4], q=[1., 0., 0., 0.])
    robot_client.close_gripper()
    robot_client.move_pose(p=[0.3, 0.0, 0.45], q=[1., 0., 0., 0.])
    robot_client.open_gripper()

def homing():
    robot_client = init()
    robot_client.move_pose(p=[0.3, 0.0, 0.5], q=[1., 0., 0., 0.])
    robot_client.open_gripper()

def close():
    robot_client = init()
    robot_client.close_gripper()

def open():
    robot_client = init()
    robot_client.open_gripper()


if __name__ == '__main__':
    main()