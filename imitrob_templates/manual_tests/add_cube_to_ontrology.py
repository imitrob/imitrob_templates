
import rclpy, time
from rclpy.node import Node
from teleop_msgs.msg import HRICommand
from crow_msgs.msg import FilteredPose, PclDimensions
from geometry_msgs.msg import Pose, Point, Quaternion

from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient

def main():
    rclpy.init()
    rosnode = Node("add_dummy_cube__node")
    soc = SceneOntologyClient(rosnode)
    
    # pub = rosnode.create_publisher(HRICommand, "/hri/command", 5)
    
    # soc.add_dummy_cube()

    '''
    # from std_msgs.msg import Float32MultiArray
    pub = rosnode.create_publisher(FilteredPose, "/filtered_poses", 5)
    pose = Pose(position=Point(x=0.0,y=0.0,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0))
    pcldim = PclDimensions(dimensions=[0.1,0.1,0.1])
    label = 'wheel'
    uuid = '586da263-0d69-447d-8a68-f51a511c4e66'
    # particle = Float32MultiArray() #std_msgs/Float32MultiArray[] 

    pubmsg = FilteredPose( 
    # This msg wraps pose of object + its label
    poses = [pose],
    size = [pcldim],
    tracked = [True],
    label = [label], #must be one of the class labels from detector
    uuid = [uuid], #identifying model in filter
    #confidence = 1.0, #confidence, can be empty
    #crow_msgs/Particles[] particles #model_particles, can be empty
    # particles = [particle]
    )

    while True:
            t = time.time()
            print(t)
            pubmsg.header.stamp.sec = int(t)
            pubmsg.header.stamp.nanosec = int(t % 1 * (10 ** 9))
            pub.publish(pubmsg)
            time.sleep(0.5)

    rclpy.spin(rosnode)

    '''
    
    print(soc.get_objects_from_onto())
    
if __name__ == '__main__':
    main()