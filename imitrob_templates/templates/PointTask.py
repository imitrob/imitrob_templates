

from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy

from teleop_msgs.msg import Intent

from imitrob_hri.imitrob_nlp.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.modules.ObjectGrounder import ObjectGrounder
from imitrob_hri.imitrob_nlp.database.Ontology import Template
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager

from typing import Dict, Any, Tuple
import json
from imitrob_templates.config import PointTaskConfig
from imitrob_templates.object_config import get_z_offset_from_center

from imitrob_templates.utils import get_quaternion_eef
from imitrob_templates.templates import BaseTask, TaskExecutionMode

class PointTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=PointTaskConfig, modes=modes, *args, **kwargs)
    
    def is_feasible(self, o, s=None):
        #assert s is None
        assert o is not None

        if ( o.properties['reachable'] # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
             ):
            return True
        else:
            return False

    
    def get_ground_data(self, relevant_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        ''' Gather all information needed to execute the task

        Args:
            relevant_data: Dictionary containing relevant data

        Returns:
            A tuple containing a boolean value indicating success and the relevant data
        '''
        # Here is listed all the data parameters to complete
        # the Task, e.g. Here single object_name
        target_object_name: str = self.target_object

        # Load target_object using its ID
        relevant_data: Dict[str, Any] = {
            'target_object': self.scene.get_object_by_name(target_object_name),
        }

        # Check once again that taget_object is detected
        assert relevant_data['target_object'] is not None

        # Returns list of grounded data
        return True, relevant_data

    def blueprint_mode_1(self, robot_client):
        '''
        Involving self vars:
            object_names (String[] or Int[]): Unique IDs of objects

        '''

        def check_preconditions(relevant_data):
            # if not check_grasped(): return False
            return True, relevant_data

        def move_1(relevant_data):
            target_object = relevant_data['target_object']
            ''' Move above the picking object '''
            p = deepcopy(np.array(target_object.absolute_location))

            p[2] += self.execution_config_params['move_near_z_offset']
            # Get Robot EEF rotation from object orientation
            q = np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN))

            # Checks if target position is correct
            # r = RealRobotConvenience.check_or_return(p, q)
            # if r == 'r': return r

            robot_client.move_pose(p, q)

            # if not RealRobotConvenience.correction_by_teleop():
            #     return 'q'
            # else:
            #     pass
            #     # print(f"target_object position corrected, diff {target_object['absolute_location'][0] - md.goal_pose.position.x}, {scene_object['absolute_location'][1] - md.goal_pose.position.y}")
            #     # Manually update target_object position, e.g.:
            #     # target_object['absolute_location'][0] = md.goal_pose.position.x
            #     # target_object['absolute_location'][1] = md.goal_pose.position.y
            return True, relevant_data

        def check_postconditions(relevant_data):
            # if check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, check_postconditions

    def mvae_mode(self, robot_client, mvae):
        raise NotImplementedError()

if __name__ == '__main__':
    task = PointTask()
    print("This is task: ")
    print(task)

"""
#!/usr/bin/env python3
from imitrob_hri.imitrob_nlp.nlp_crow.database.Ontology import Template

from imitrob_hri.imitrob_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.nlp_crow.modules.ObjectGrounder import ObjectGrounder
from imitrob_hri.imitrob_nlp.nlp_crow.modules.PositionStorageGrounder import PositionStorageGrounder
from imitrob_hri.imitrob_nlp.nlp_crow.modules.PositionStorageDetector import PositionStorageDetector
from imitrob_hri.imitrob_nlp.nlp_crow.structures.tagging.TaggedText import TaggedText
from imitrob_hri.imitrob_nlp.nlp_crow.modules.UserInputManager import UserInputManager
from crow_msgs.msg import CommandType

import logging

class PointTask(Template):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lang = 'cs'
        self.ui = UserInputManager(language = self.lang)
        self.templ_det = self.ui.load_file('templates_detection.json')
        self.parameters = ['action', 'action_type', 'target_type']
        self.target = [] # object to point
        self.position = None
        self.storage = None
        self.target_type = 'onto_uri'
        self.action_type = self.templ_det[self.lang]['point']
        self.action = CommandType.POINT
        self.logger = logging.getLogger(__name__)
        
        
        self.name = 'point'
        self.compare_types = ['template', 'selections']
        self.complexity = 1

    def match(self, tagged_text : TaggedText, language = 'en', client = None) -> None:
        psd = PositionStorageDetector(language=language, client = client)
        self.position = psd.detect_position(tagged_text)
        self.storage = psd.detect_storage(tagged_text)
        if self.position:
            self.parameters.append('position')
        elif self.storage:
            self.parameters.append('storage')
        else:
            od = ObjectDetector(language = language, client = client)
            self.target = od.detect_object(tagged_text)
            self.parameters.append('target')

    def evaluate(self, language = 'en', client = None) -> None:
        # check if the object to be put down is in the workspace
        self.lang = language
        self.ui = UserInputManager(language = self.lang)
        self.guidance_file = self.ui.load_file('guidance_dialogue.json')
        og = ObjectGrounder(language = self.lang, client = client)
        if self.target:
            self.target, self.target_ph_cls, self.target_ph_color, self.target_ph_loc = og.ground_object(obj_placeholder=self.target)
            names_to_add = ['target_ph_cls', 'target_ph_color', 'target_ph_loc']
            for name in names_to_add:
                if getattr(self, name):
                    self.parameters.append(name)
        elif self.position:
            pg = PositionStorageGrounder(language = self.lang, client = client)
            self.target = pg.ground_position(self.position)
            self.parameters.append('target')
        elif self.storage:
            pg = PositionStorageGrounder(language = self.lang, client = client)
            self.target = pg.ground_storage(self.storage)
            self.parameters.append('target')

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False

    def task_property_penalization_selections(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 1.0,
                'pickable':  1.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':1.0,
                'full-liquid':1.0,
                'glued':     1.0,
            }[property]

    def is_feasible(self, o=None, s=None):
        assert o is not None 
        #assert s is None

        return True
"""