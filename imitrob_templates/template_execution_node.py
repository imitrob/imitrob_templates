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
from imitrob_templates.small_template_factory import create_template

class HRICommandRunnerNode(Node):
    def __init__(self, rosnode_name = 'HRI_runner'):
        super().__init__(rosnode_name)
        
        self.robot_client = RobotActionClient(self)
        self.robot_client.start()
        
        self.oc = OntologyClientAdHoc(self)
        
        s = self.oc.get_updated_scene()

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
        task.match_intent(i, self.oc.get_updated_scene())
        #task.ground(s=self.oc.get_objects_from_onto())

        print(self.oc.get_updated_scene())
        task.target_object = target_object
        task.ground_realpositions(self.oc.get_updated_scene())
        ## tihs iwll be in grounder

        print("********************")
        for k, v in task.__dict__.items():
            print(f"{k}: {v}")
        print("********************")

        ret_bool, ret = task.execute(self.robot_client, mode=TaskExecutionMode.BASIC)
        
        print(f"handle_hricommand ended: {ret_bool}, message: {ret}")
        


''' Just for the test '''
class OntologyClientAdHoc():
    def __init__(self, rosnode):
        self.crowracle = CrowtologyClient(node=rosnode)
        self.onto = self.crowracle.onto
        
        # self.add_dummy_cube()
        # self.add_dummy_cube()
        # print(self.get_objects_from_onto())
        print("Ontology Client Ready")
        
    @staticmethod
    def mocked_update_scene():
        s = None
        s = Scene2(init='object', random=False)
        sl.scene = s
        return s


    NAME2TYPE = {
        'cube': 'Object',
        
    }

    def get_updated_scene(self):
        
        s = None
        s = Scene2(init='', objects=[], random=False)
        
        objects = self.get_objects_from_onto()
        ''' object list; item is dictionary containing uri, id, color, color_nlp_name_CZ, EN, nlp_name_CZ, nlp_name_EN; absolute_location'''

        '''
        import rdflib
        uri = rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551')
        '''
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]

        # Colors:
        # [COLOR_GREEN. 

        for object in objects:
            ''' o is dictionary containing properties '''
            uri = object['uri']
            id = object['id']

            # created name `wheel_od_0` extracted from 'http://imitrob.ciirc.cvut.cz/ontologies/crow#wheel_od_0'
            name = str(object['uri']).split("#")[-1]

            color = object['color']
            color_nlp_name_CZ = object['color_nlp_name_CZ']
            color_nlp_name_EN = object['color_nlp_name_EN']
            nlp_name_CZ = object['nlp_name_CZ']
            nlp_name_EN = object['nlp_name_EN']
            absolute_location = object['absolute_location']
            
            o = Object2(name=name, position_real=np.array(absolute_location), random=False)
            # o.quaternion = np.array(object['pose'][1])
            # o.color_uri = color
            o.color = color_nlp_name_EN
            # o.color_name_CZ = color_nlp_name_CZ
            
            o.nlp_name_CZ = nlp_name_CZ
            o.nlp_name_EN = nlp_name_EN
            o.crow_id = id
            o.crow_uri = uri
            
            s.objects.append(o)

        return s

    def match_object_in_onto(self, obj):
        onto = self.get_objects_from_onto()
        obj = json.loads(obj)
        for o in onto:
           url = str(o["uri"])
           if url is not None and url == obj[0]["target"][0]:
              return o
        return None
        
        
    def add_dummy_cube(self):
        o_list = self.get_objects_from_onto()
        if len(o_list) == 0:
           print("Adding a dummy cube into ontology")
           self.crowracle.add_test_object("CUBE")
        
    def get_objects_from_onto(self):
        o_list = self.crowracle.getTangibleObjectsProps()
        #print("Onto objects:")
        #print(o_list)
        return o_list
        
    def get_action_from_cmd(self, cmd):
        action = json.loads(cmd)[0]["action_type"].lower()
        if action in ACTION_TRANSL.keys():
           return ACTION_TRANSL[action]
        else:
           print("No czech translation for action " + action)
           return action
        
        
    def publish_trajectory(self, trajectory):
        #TODO choose proper msg type
        action = json.dumps(trajectory)
        msg = StampedString()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = action
        print(f'Publishing {msg.data}')
        self.trajectory_publisher.publish(msg)
        

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