import rclpy, json
from rclpy.node import Node
#from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
#                                   ReentrantCallbackGroup)
#from rclpy.executors import MultiThreadedExecutor

import numpy as np

from crow_ontology.crowracle_client import CrowtologyClient
from imitrob_robot_client.robot_client import RobotActionClient
from teleop_msgs.msg import Intent, HRICommand

from context_based_gesture_operation.srcmodules.Scenes import Scene as Scene2
from context_based_gesture_operation.srcmodules.Objects import Object as Object2
from imitrob_templates.templates.BaseTask import TaskExecutionMode
from imitrob_hri.imitrob_nlp.TemplateFactory import create_template
from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient

import time


class HRICommandRunnerNode(Node):
    def __init__(self, rosnode_name = 'HRI_runner'):
        super().__init__(rosnode_name)
        
        self.robot_client = RobotActionClient(self)
        self.robot_client.start()
        
        self.oc = SceneOntologyClient(self)
        
        s = self.oc.get_scene2()

        self.hricommand_queue = []
        self.hric_sub = self.create_subscription(HRICommand,
            '/hri/command',
            self.add_to_queue, 5) #, callback_group=MutuallyExclusiveCallbackGroup())


        self.scene = s
        print(self.scene.get_object_by_name('cube_holes_od_0'))

        print("Done")

    def add_to_queue(self, msg):
        self.hricommand_queue.append(msg)

    def handle_hricommand(self, msg):
        """Single HRICommand linked to single Template

        Args:
            msg (HRICommand): Description of Command to execute

        Returns:
            bool: Success
        """
        print("handle_hricommand started")

        receivedHRIcommandStringToParse = msg.data[0]
        receivedHRIcommand_parsed = json.loads(receivedHRIcommandStringToParse)
        
        # Extract data
        target_action = receivedHRIcommand_parsed['target_action']

        ### TEMPORARY REMMAPPING OF ACTIONS, PLEASE DELETE THIS
        # if 'put' in target_action:   
        #     msg = self.run_template(target_action='open', object_name='drawer_socket')
        #     receivedHRIcommandStringToParse = msg.data[0]
        #     receivedHRIcommand_parsed = json.loads(receivedHRIcommandStringToParse)
        #     target_action = receivedHRIcommand_parsed['target_action']
        # if 'place' in target_action:   
        #     msg = self.run_template(target_action='close', object_name='drawer_socket')
        #     receivedHRIcommandStringToParse = msg.data[0]
        #     receivedHRIcommand_parsed = json.loads(receivedHRIcommandStringToParse)
        #     target_action = receivedHRIcommand_parsed['target_action']
        ###################################

        target_object = receivedHRIcommand_parsed['target_object']
        
        # Possibility to use all data (probabilities, timestamps, etc.)
        # template_names = receivedHRIcommand_parsed['actions']
        # template_probs = receivedHRIcommand_parsed['action_probs']
        # template_timestamps = receivedHRIcommand_parsed['action_timestamp']
        
        # object_names = receivedHRIcommand_parsed['objects']
        # object_probs = receivedHRIcommand_parsed['object_probs']
        # object_classes = receivedHRIcommand_parsed['object_classes']
        
        # parameters = receivedHRIcommand_parsed['parameters']

        i = Intent()
        i.target_action = target_action
        i.target_object = target_object #'cube_holes_od_0' #s.objects[0].name
        

        task = create_template(i.target_action)
        if task is None: return # template not found; quitting
        
        #task.match_intent(i, self.oc.crowracle)
        scene2 = self.oc.get_scene2()
        # task.match_intent(i, scene2)
        task.match_parsed_hricommand(receivedHRIcommand_parsed, scene2)
        #task.ground(s=self.oc.get_objects_from_onto())

        print(scene2)
        task.ground_scene(scene2)
        ## tihs iwll be in grounder

        print("********************")
        for k, v in task.__dict__.items():
            print(f"{k}: {v}")
        print("********************")

        ret_bool, ret = task.execute(self.robot_client, self.oc, mode=TaskExecutionMode.BASIC)
        
        print(f"handle_hricommand ended: {ret_bool}, message: {ret}")
        

    ### TEMPORARY REMMAPPING OF ACTIONS, PLEASE DELETE THIS
    def run_template(self, target_action, object_name='cube_holes'):

        if target_action not in ['release']:
            print("Need to ground scene object:")

            scene = self.oc.get_scene2()
            target_object = None
            for o_name in scene.O:
                if object_name in o_name: # e.g.: 'cube_holes_od_0'
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
        
        return msg_to_send
    ###################################

def main():
    rclpy.init()
    n_threads = 20 # nr of callbacks in group, +1 as backup
    #mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())

    rosnode = HRICommandRunnerNode()
    while rclpy.ok():
        rclpy.spin_once(rosnode) #, executor=mte)
        # check new execution
        if len(rosnode.hricommand_queue) > 0:
            msg = rosnode.hricommand_queue.pop(0)
            rosnode.handle_hricommand(msg)

if __name__ == '__main__':
    main()