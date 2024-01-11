
import rclpy
from rclpy.node import Node
from imitrob_robot_client.robot_client import RobotActionClient


def init():
    rclpy.init()
    rosnode = Node("test_robot_moving_node")
    robot_client = RobotActionClient(rosnode)
    robot_client.start()
    return robot_client

def test_outofreach_error():
    robot_client = init()
    for z in [0.5]:
        robot_client.move_pose(p=[0.8, 0.0, z], q=[1., 0., 0., 0.])
    
    print("test_outofreach_error PASSED")


if __name__ == '__main__':
    test_outofreach_error()