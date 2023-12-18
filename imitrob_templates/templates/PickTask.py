import json
from copy import deepcopy
from typing import Any, Dict, Tuple

import numpy as np

from crow_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
from imitrob_templates.config import PickTaskConfig
from imitrob_templates.object_config import get_z_offset_from_center
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from imitrob_templates.utils import get_quaternion_eef
from teleop_msgs.msg import Intent


class PickTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=PickTaskConfig, modes=modes, *args, **kwargs)

    def is_feasible(self, o, s=None):
        # assert s is None
        assert o is not None

        if (o.properties['reachable']  # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
            and o.properties['pickable']  # When object is not pickable it cannot be picked at all
            and not o.properties['glued']):
            return True
        else:
            return False

    ''' There will be function that converts tagged text to HRICommand object '''
    @staticmethod
    def tagged_text_to_HRICommand(self, tagged_text):
        pass
    

    def match_tagged_text(self, tagged_text : Intent, language = 'en', client = None) -> bool:
        ''' TODO: tagged_text is output from sentence processor
            Checks if given command TaggedText corresponds to this template without checking the current detection
            Checks general classes in ontology (not in real world instances) 
        (Idea: Runs in NLP package and then in modality merger)

        Returns:
            match (Bool):
        '''
        od = ObjectDetector(language = language, client = client)
        self.target = od.detect_object(tagged_text)

        # # 1. Load all trigger words (verbs) for this template
        # trigger_words = template_name_synonyms[self.id]
        # # 2. Occurance
        # if tagged_text.action in trigger_words:
        #     matched = True
        # else:
        #     matched = False

        # if matched:
        #     return True
        # else:
        #     return False

    def match_intent(self, intent : Intent, scene) -> bool:
        ''' Checks general classes in ontology (not in real world instances) 
        (Idea: Runs in NLP package and then in modality merger)
        
        Returns:
            match (Bool): 
        '''

        # is object on the scene?
        if scene.get_object_by_name(intent.target_object) is None:
            return False

        self.target_object = intent.target_object
        self.target_action = intent.target_action

        return True


    def ground(self, language = 'en', client = None):
        ''' Grounding on the real objects
        (Runs in Modality Merger)
        '''

        return

    def ground_realpositions(self, s):
        self.scene = s

    @staticmethod
    def get_ground_data(relevant_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
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

        def check_postconditions(relevant_data):
            # if check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, move_2, move_3, check_postconditions

    def mvae_mode(self, robot_client, mvae):

        def infer(relevant_data, mvae=mvae):
            target_object = relevant_data['target_object']
            mvae_cmd = mvae.get_action_from_cmd(json.dumps([{"action_type": self.name}]))
            joints, _ = mvae.mvae_infer(mvae_cmd, target_object.absolute_location)

            relevant_data = {
                "traj": joints
            }
            return True, relevant_data

        def move(relevant_data):
            trajectory, gripper = relevant_data["traj"][:-1], relevant_data["traj"][-1]
            robot_client.trajectory_joint(trajectory, gripper)

        return self.get_ground_data, infer, move


if __name__ == '__main__':
    task = PickTask()
    print("This is task: ")
    print(task)
