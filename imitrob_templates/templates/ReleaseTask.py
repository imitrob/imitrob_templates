
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy

from teleop_msgs.msg import Intent

from imitrob_hri.imitrob_nlp.modules.ObjectDetector import ObjectDetector

from typing import Dict, Any, Tuple
import json
from imitrob_templates.config import ReleaseTaskConfig
from imitrob_templates.object_config import get_z_offset_from_center

from imitrob_templates.utils import get_quaternion_eef
from imitrob_templates.templates import BaseTask, TaskExecutionMode

class ReleaseTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 0
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=ReleaseTaskConfig, modes=modes, *args, **kwargs)
    
    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        if tagged_text.contains_text(templ_det[lang]['release']):
            return True, 'release'
        else:
            return False, ''

    def is_feasible(self, o, s=None):
        #assert s is None
        # assert o is not None

        assert super().is_feasible(o,s) == True
        return True
        
    def match_tagged_text(self, tagged_text : Intent, language = 'en', client = None) -> bool:
        # used by NLP processor
        raise NotImplementedError()

    def ground(self, language = 'en', client = None):
        # used by NLP processor
        raise NotImplementedError()

    
    def get_ground_data(self, relevant_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        ''' Gather all information needed to execute the task

        Args:
            relevant_data: Dictionary containing relevant data

        Returns:
            A tuple containing a boolean value indicating success and the relevant data
        '''

        # Load target_object using its ID
        relevant_data: Dict[str, Any] = {
            'free_space': self.scene.position_real(self.scene.get_random_position_in_scene2()),
        }

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
            free_space = relevant_data['free_space']
            ''' Move to free space '''
            p = deepcopy(np.array(free_space))
            q = np.array([1.,0.,0.,0.])
            p[2] += self.execution_config_params['free_space_z_offset']
            
            robot_client.move_pose(p, q)
            robot_client.open_gripper()

            return True, relevant_data

        def check_postconditions(relevant_data):
            # if check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, check_postconditions

if __name__ == '__main__':
    task = ReleaseTask()
    print("This is task: ")
    print(task)
