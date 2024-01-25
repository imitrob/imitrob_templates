import rclpy, time, sys
from rclpy.node import Node
from teleop_msgs.msg import HRICommand
from crow_msgs.msg import FilteredPose, PclDimensions
from geometry_msgs.msg import Pose, Point, Quaternion

from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient

def main():
    rclpy.init()
    rosnode = Node("add_dummy_cube__node")
    soc = SceneOntologyClient(rosnode)
    
    pub = rosnode.create_publisher(HRICommand, "/hri/command", 5)
    
    soc.add_dummy_cube()
    
    print(soc.get_objects_from_onto())

def scene_experimental_setup_node(objects={
    'wheel': {
        'pose': Pose(position=Point(x=0.2,y=0.0,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': '586da263-0d69-000d-8a68-f51a511c4e66', 
        'size': PclDimensions(dimensions=[0.1,0.1,0.1]),
        'tracked': True,
    },
    'cube_holes': {
        'pose': Pose(position=Point(x=0.1,y=0.0,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': '586da263-0d69-001d-8a68-f51a511c4e66', 
        'size': PclDimensions(dimensions=[0.1,0.1,0.1]),
        'tracked': True,            
    },
    }):
    """ object_adder_node
    Constantly publishes predefined set of objects to the ontology to fulfill test

    Args:
        objects (dict, optional): _description_. 
    """    
    
    poses = []
    sizes = []
    uuids = []
    tracked = []
    labels = []
    for obj_key in objects.keys():
        obj = objects[obj_key]
        poses.append(obj['pose'])
        sizes.append(obj['size'])
        uuids.append(obj['uuid'])
        tracked.append(obj['tracked'])
        labels.append(obj_key)

    pubmsg = FilteredPose(poses=poses,size=sizes,tracked=tracked,label=labels,uuid =uuids)
    
    rclpy.init()
    rosnode = Node("adder_tester")
    pub = rosnode.create_publisher(FilteredPose, "/filtered_poses", 5)
    
    while True:
            pubmsg.header.stamp = rosnode.get_clock().now().to_msg()
            pub.publish(pubmsg)
            time.sleep(0.5)

    rclpy.spin(rosnode)

if __name__ == '__main__':
    if len(sys.argv[1]) > 1:
        eval(sys.argv[1]+'()')
    else:
        main()