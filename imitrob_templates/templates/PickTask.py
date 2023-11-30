

# from imitrob_hri.imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy
from config import PickTaskConfig
import ycb_data

from utils import get_quaternion_eef

from teleop_msgs.msg import Intent

class PickTask():
    def __init__(self):
        self.name = PickTaskConfig['name']
        self.id = PickTaskConfig['id']
        self.pars_compulsary = PickTaskConfig['pars_compulsary']
        
        self.n_target_objects = 1
        
        self.target_object_penalizatiion = PickTaskConfig['target_object_penalization']
        self.execution_config_params = PickTaskConfig['execution_config_params']
        
        # REVISE: Will it be saved 
        self.grounded_vars = {}
    
    @property
    def compare_types(self):
        return self.pars_compulsary
        
    def complexity(self):
        return self.compare_types - 1 # complexity = 1

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False

    def task_property_penalization_target_objects(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return self.target_object_penalization[property]
    
    def is_feasible(self, o, s=None):
        #assert s is None
        assert o is not None

        if ( o.properties['reachable'] and  # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
             o.properties['pickable'] and  # When object is not pickable it cannot be picked at all
             not o.properties['glued'] ):
            return True
        else:
            return False
        
    ''' There will be function that converts tagged text to HRICommand object '''
    @staticmethod
    def tagged_text_to_HRICommand(self, tagged_text):
        pass
    
    def match(self, tagged_text : Intent, language = 'en', client = None) -> None:
        ''' Checks if given command TaggedText corresponds to this template without checking the current detection
            Checks general classes in ontology (not in real world instances) 
        (Idea: Runs in NLP package and then in modality merger)
        
        Returns:
            match (Bool): 
        '''
        # 1. Load all trigger words (verbs) for this template
        # trigger_words = template_name_synonyms[self.id]
        trigger_words = ['pick', 'PICK_TASK', 'seber', 'pick', 'PickTask']
        # 2. Occurance
        if tagged_text.action in trigger_words:
            matched = True
        else:
            matched = False
            
        if matched:
            return True
        else:
            return False
        
    def ground(self, i, s):
        ''' Grounding on the real objects 
        (Runs in Modality Merger)
        '''

        # I don't know what will be the format of the scene retrived from the ontology
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]
        
        # Finds i.target_object in the ontology objects
        # Input:
        i.target_object
        s
        
        self.grounded_vars['object_names'] = s[0]['id']
        
        return s[0] # tmp

        
    def blueprint_mode_1(self, robot_client):
        '''
        Involving self vars:
            object_names (String[] or Int[]): Unique IDs of objects
            
        '''
        object_names = self.grounded_vars['object_names']
        
        def check_preconditions():
            if len(object_names) != self.n_target_objects: return False
            
            # if not check_grasped(): return False
            
            return True
        
        def get_ground_data(s):
            '''  
            Args:    
                s (Scene)
            Returns:
                parameters e.g. scene_object (object)
            '''
            # Here is listed all the data parameters to complete
            # the Task, e.g. Here single object_name
            target_object_name = object_names[0]
            
            # Load scene_object using its ID
            def get_object_by_name(target_object_name):
                for scene_object in s:
                    if scene_object['id'] == target_object_name:
                        return scene_object
                raise Exception("Handle object not visible now")
            
            grounded_data = {
                'scene_object': get_object_by_name(target_object_name),
            }
            # Returns list of grounded data
            return grounded_data
        
        def move_1(relevant_data):
            scene_object = relevant_data['scene_object']
            ''' Move above the picking object '''
            p = deepcopy(np.array(scene_object['absolute_location']))
            
            p[2] += self.execution_config_params['move_near_z_offset']
            # Get Robot EEF rotation from object orientation
            q = np.array(get_quaternion_eef(scene_object.quaternion, scene_object.name))
            
            # Checks if target position is correct
            # r = RealRobotConvenience.check_or_return(p, q)
            # if r == 'r': return r
            
            robot_client.open_gripper()
            robot_client.move_pose(p, q)
            
            # if not RealRobotConvenience.correction_by_teleop():
            #     return 'q'
            # else:
            #     pass
            #     # print(f"scene_object position corrected, diff {scene_object['absolute_location'][0] - md.goal_pose.position.x}, {scene_object['absolute_location'][1] - md.goal_pose.position.y}")
            #     # Manually update scene_object position, e.g.:
            #     # scene_object['absolute_location'][0] = md.goal_pose.position.x
            #     # scene_object['absolute_location'][1] = md.goal_pose.position.y
            return True, relevant_data
        
        def move_2(relevant_data):
            scene_object = relevant_data['scene_object']
            ''' Move to object grasp point '''
            p = deepcopy(np.array(scene_object['absolute_location']))
            p[2] += ycb_data.OFFSETS[scene_object['id']]
            q = deepcopy(np.array(get_quaternion_eef(scene_object['quaternion'], scene_object['id'])))
            # r = RealRobotConvenience.check_or_return(p, q)
            # if r == 'r': return r
            robot_client.move_pose(p, q)
            robot_client.close_gripper()
            # Pragmatic: Pick scene_object should have included some time delay
            # time.sleep(2.)
            relevant_data['p'], relevant_data['q'] = p, q
            return True, relevant_data
            
        def move_3(relevant_data):
            scene_object = relevant_data['scene_object']
            p, q = relevant_data['p'], relevant_data['q']
            ''' Move up littlebit '''
            p[2] += self.execution_config_params['move_final_z_offset']
            robot_client.move_pose(p, q)
            
            return True, relevant_data

        def check_postconditions():
            # if check_grasped(): return False

            return True

        return check_preconditions, get_ground_data, move_1, move_2, move_3, check_postconditions

    def execute(self, mode=1):
        if mode == 1:
            steps = self.blueprint_mode_1()
        else:
            raise Exception(NotImplementedError)
        
        relevant_data = {}
        for n,step in enumerate(steps):
            ret, relevant_data = step(relevant_data)
            if ret == False: return 'Failure'
        