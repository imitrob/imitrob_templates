from typing import Dict, Any, Tuple
import json
import time
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy
from imitrob_templates.config import PassTaskConfig
from imitrob_templates.object_config import get_z_offset_from_center

from imitrob_templates.utils import get_quaternion_eef
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from teleop_msgs.msg import Intent

from crow_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector


class PassTask(BaseTask):

    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=PassTaskConfig, modes=modes, *args, **kwargs)
    
    def is_feasible(self, o, s=None):
        #assert s is None
        assert o is not None

        if ( o.properties['reachable'] and  # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
             o.properties['pickable'] and  # When object is not pickable it cannot be picked at all
             not o.properties['glued'] ):
            return True
        else:
            return False

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

            robot_client.open_gripper()
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

        def move_2(relevant_data):
            target_object = relevant_data['target_object']
            ''' Move to object grasp point '''
            p = deepcopy(np.array(target_object.absolute_location))
            p[2] += get_z_offset_from_center(target_object.name)
            q = deepcopy(np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN)))
            # r = RealRobotConvenience.check_or_return(p, q)
            # if r == 'r': return r
            robot_client.move_pose(p, q)
            robot_client.close_gripper()

            # Pragmatic: Pick target_object should have included some time delay
            # time.sleep(2.)
            relevant_data['p'], relevant_data['q'] = p, q
            return True, relevant_data

        def move_3(relevant_data):
            p, q = relevant_data['p'], relevant_data['q']
            ''' Move up littlebit '''
            p[2] += self.execution_config_params['move_final_z_offset']

            robot_client.move_pose(p, q)

            return True, relevant_data
        
        def move_4(relevant_data):
            p, q = self.execution_config_params['pass_to_position'], relevant_data['q']

            robot_client.move_pose(p, q)

            return True, relevant_data

        def check_postconditions(relevant_data):
            # if check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, move_2, move_3, move_4, check_postconditions

    def mvae_mode(self):
        raise Exception("TODO")


        
if __name__ == '__main__':
    task = PassTask()
    print("This is task: ")
    print(task)
