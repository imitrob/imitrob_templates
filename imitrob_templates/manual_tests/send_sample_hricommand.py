
import rclpy, time
from rclpy.node import Node
from teleop_msgs.msg import HRICommand

from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient

class PublishHRICommand(Node):
    def __init__(self):
        super().__init__("rosnodePublisherOfHRICommand")
        
        self.pub = self.create_publisher(HRICommand, "/hri/command", 5)

        self.soc = SceneOntologyClient(self)



def run_template(rosnode, target_action):

    if target_action not in ['release']:
        print("Need to ground scene object:")

        scene = rosnode.soc.get_scene2()
        target_object = None
        for o_name in scene.O:
            if 'cube_holes' in o_name: # e.g.: 'cube_holes_od_0'
                target_object = o_name
                break
        if target_object is None:
            print("cube (cube_holes) not found on the scene! Waiting and trying again")
            time.sleep(5)
            return
        else:
            print(f"cube chosen ({target_object})")

    else:
        print("Don't need to ground scene object")
        target_object = ''

        msg_to_send = HRICommand()
        s = "{'target_action': '" + target_action + "', 'target_object': '" + target_object + "', "
        s += "'actions': ['" + target_action + "', 'release', 'pass', 'point'], " #"', 'move_left', 'move_down', 'move_right', 'pick_up', 'put', 'place', 'pour', 'push', 'replace'], "
        s += "'action_probs': ['" + "1.0" + "', '0.05', '0.1', '0.15'], " #", 0.014667431372768347, 0.0008680663118536268, 0.035168211530459945, 0.0984559292675215, 0.012854139530004692, 0.0068131722011598225, 0.04846120672655781, 0.0020918881693065285, 0.01454853390045828], "
        
        if target_object == '':
            scene_objects = "[]"
            scene_object_probs = "[]"
            scene_object_classes = "[]"
        else:
            scene_objects = "['" + target_object + "', 'wheel', 'sphere']"
            scene_object_probs = "[1.0, 0.1, 0.15]"
            scene_object_classes = "['object']"
        
        s += "'action_timestamp': 0.0, 'objects': " + scene_objects + ", "
        s += "'object_probs': " + scene_object_probs + ", 'object_classes': " + scene_object_classes + ", 'parameters': ''}"
        s = s.replace("'", '"')
        msg_to_send.data = [s]
        
        rosnode.pub.publish(msg_to_send)

def send_templates_1():
    rclpy.init()
    rosnode = PublishHRICommand()

    templates_to_run = ['point', 'pick', 'release', 'pass']
    for template in templates_to_run:
        input(f"Run template {template} (ENTER)")
        run_template(rosnode, template)

def main():
    """ User Input assign template, infinite loop
    """    

    rclpy.init()
    print("This is imitrob_template tester")
    rosnode = PublishHRICommand()
    print("This is imitrob_template tester")
    print("- This exmaple uses crow object cube_holes!")

    while True:
        target_action = str(input("Enter task name: "))
        run_template(rosnode, target_action)


if __name__ == '__main__':   
    main()