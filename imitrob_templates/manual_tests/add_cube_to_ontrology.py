
import rclpy, time
from rclpy.node import Node
from teleop_msgs.msg import HRICommand

from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient

def main():
    rclpy.init()
    rosnode = Node("add_dummy_cube__node")
    soc = SceneOntologyClient(rosnode)
    
    # pub = rosnode.create_publisher(HRICommand, "/hri/command", 5)
    
    soc.add_dummy_cube()
    
    print(soc.get_objects_from_onto())
    
if __name__ == '__main__':
    main()