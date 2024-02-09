from typing import Dict, Any, Tuple
import json
import time
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy
from imitrob_templates.config import CloseTaskConfig, NonMovePartRelations, TEMPORARY_VISION_ERROR_CORRECTION_POINT
from imitrob_templates.object_config import get_z_offset_from_center

from imitrob_templates.utils import get_quaternion_eef
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from imitrob_templates.templates.OpenTask import OpenTask
from teleop_msgs.msg import Intent

from imitrob_hri.imitrob_nlp.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.modules.ObjectGrounder import ObjectGrounder
from imitrob_hri.imitrob_nlp.database.Ontology import Template
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager
from crow_msgs.msg import CommandType
from scipy.spatial.transform import Rotation as R

from spatialmath import UnitQuaternion
import spatialmath as sm

class CloseTask(OpenTask):

    def __init__(self, *args, **kwargs):
        super().__init__(task_config=CloseTaskConfig, *args, **kwargs)


    def blueprint_mode_1(self, robot_client, ontology_client):
        def check_preconditions(relevant_data):
            # if not check_grasped(): return False
            return True, relevant_data

        def move_1(relevant_data):
            ''' Move above the picking object '''
            p, q = self.get_handle_pose(relevant_data)

            ''' move above '''
            p[2] += self.execution_config_params['move_near_z_offset']

            # Get Robot EEF rotation from object orientation
            #q = np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN))
            
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
            ''' Move to object grasp point '''
            p, q = self.get_handle_pose(relevant_data)

            # r = RealRobotConvenience.check_or_return(p, q)
            # if r == 'r': return r
            robot_client.move_pose(p, q)
            robot_client.close_gripper()

            # Pragmatic: Pick target_object should have included some time delay
            # time.sleep(2.)
            return True, relevant_data

        def move_3(relevant_data):
            ''' Move closed! '''
            p, q = self.get_drawer_pose_at_state(relevant_data, drawer_state='close')
            
            robot_client.move_pose(p, q)
            # robot_client.open_gripper()

            return True, relevant_data

        def move_4(relevant_data):
            # p, q = relevant_data['p'], relevant_data['q']
            ''' Move up littlebit '''
            # p[2] += self.execution_config_params['move_final_z_offset']

            robot_client.open_gripper()
            robot_client.move_pose(p=[0.5,0.0,0.3], q=[1.0,0.0,0.0,0.0])

            return True, relevant_data

        def check_postconditions(relevant_data):
            # robot needs to move out for the camera to see the marker
            time.sleep(2.0)
            
            openness_data = ontology_client.crowracle.read_drawer_openness_level()
            
            # choose the right drawer
            openness_level = None
            drawer_name = relevant_data['target_opened_object'].name
            for d in openness_data:
                if str(d[0].fragment) == drawer_name:
                    openness_level = float(d[1])
                    break
            
            if openness_level is None:
                print("WARNING: Drawer not found")
                return False, relevant_data

            is_open = openness_level > 0.5
            if is_open: 
                return False, relevant_data
            else:
                return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, move_2, move_3, move_4, check_postconditions
