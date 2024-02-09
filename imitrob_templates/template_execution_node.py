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