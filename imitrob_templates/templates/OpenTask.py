from typing import Dict, Any, Tuple
import json
import time
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms
import numpy as np
from copy import deepcopy
from imitrob_templates.config import OpenTaskConfig, NonMovePartRelations, TEMPORARY_VISION_ERROR_CORRECTION_POINT
from imitrob_templates.object_config import get_z_offset_from_center

from imitrob_templates.utils import get_quaternion_eef
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from teleop_msgs.msg import Intent

from imitrob_hri.imitrob_nlp.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.modules.ObjectGrounder import ObjectGrounder
from imitrob_hri.imitrob_nlp.database.Ontology import Template
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager
from crow_msgs.msg import CommandType
from scipy.spatial.transform import Rotation as R

from spatialmath import UnitQuaternion
import spatialmath as sm

class OpenTask(BaseTask):

    def __init__(self, task_config=OpenTaskConfig, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=task_config, modes=modes, *args, **kwargs)

    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        
        if tagged_text.contains_text(templ_det[lang]['open']):
            return True, 'open'
        else:
            return False, ''

    def is_feasible(self, o, s=None):
        ''' Might be deleted in the future, overwritten by general BaseTask:is_feasible() '''
        #assert s is None
        assert o is not None

        ret = None
        if ( o.properties['reachable'] and  # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
             o.properties['pickable'] and  # When object is not pickable it cannot be picked at all
             not o.properties['glued'] ):
            ret = True
        else:
            ret = False

        assert super().is_feasible(o,s) == ret
        return ret
    
    def get_ground_data(self, relevant_data: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
        ''' Gather all information needed to execute the task
        Args:
            relevant_data: Dictionary containing relevant data
        Returns:
            A tuple containing a boolean value indicating success and the relevant data
        '''
        
        # target_opened_object - drawer - can be both target_object or target_storage 
        for opened_object in [self.target_object, self.target_storage]:
            target_opened_object_name = opened_object

            target_opened_object = self.scene.get_object_by_name(target_opened_object_name)
            # find 'drawer_cabinet non-movable part related to drawer_socket moving part
            non_move_part_type = NonMovePartRelations[target_opened_object.type]

            # Load target_object using its ID
            relevant_data: Dict[str, Any] = {
                'target_opened_object': target_opened_object,
                'target_drawer_cabinet': self.scene.get_object_by_type(non_move_part_type),
            }

            if relevant_data['target_opened_object'] is not None: break

        # Check once again that target_opened_object is detected
        assert relevant_data['target_opened_object'] is not None

        # Returns list of grounded data
        return True, relevant_data

    def target_opened_object_to_handle(self, target_opened_object):
        ''' Note: Drawer is directed in y axis
            That's why center_to_handle added to y axis
        '''

        center_to_handle = self.execution_config_params['drawer_center_to_handle']

        R_robot = R.from_quat(target_opened_object.quaternion)
        T_base_openedobject = np.eye(4)
        T_base_openedobject[:3, :3] = R_robot.as_matrix()
        T_base_openedobject[:3, 3] = np.array(target_opened_object.absolute_location)

        T_openedobject_handle = np.eye(4)
        T_openedobject_handle[:3, 3] = np.array([0.0, center_to_handle, 0.0]) # measured to center socket to handle

        T_base_openedobjecthandle = np.matmul(T_base_openedobject, T_openedobject_handle)
        T_base_openedobjecthandle_rotvec = R.from_rotvec(T_base_openedobjecthandle[:3, :3])

        return T_base_openedobjecthandle[:3, 3], T_base_openedobjecthandle_rotvec.as_quat(canonical=False)

    def target_cabinet_to_max_opened_handle(self, drawer_cabinet):
        ''' Note: Drawer is directed in y axis 
            That's why center_to_handle+max_gauge added to y axis
        '''
        # Get how much it can open
        max_gauge = self.execution_config_params['drawer_open_to_close_dist']
        
        center_to_handle = self.execution_config_params['drawer_center_to_handle']

        R_robot = R.from_quat(drawer_cabinet.quaternion)
        T_base_openedobject = np.eye(4)
        T_base_openedobject[:3, :3] = R_robot.as_matrix()
        T_base_openedobject[:3, 3] = np.array(drawer_cabinet.absolute_location)

        T_openedobject_maxopenedhandle = np.eye(4)
        T_openedobject_maxopenedhandle[:3, 3] = np.array([0.0, center_to_handle + max_gauge, 0.0]) # measured to center socket to handle

        T_base_maxopenedobjecthandle = np.matmul(T_base_openedobject, T_openedobject_maxopenedhandle)
        T_base_maxopenedobjecthandle_rotvec = R.from_rotvec(T_base_maxopenedobjecthandle[:3, :3])

        return T_base_maxopenedobjecthandle[:3, 3], T_base_maxopenedobjecthandle_rotvec.as_quat(canonical=False)

    def target_cabinet_to_min_opened_handle(self, drawer_cabinet):
        ''' Note: Drawer is directed in y axis 
            That's why center_to_handle+max_gauge added to y axis
        '''
        center_to_handle = self.execution_config_params['drawer_center_to_handle']

        R_robot = R.from_quat(drawer_cabinet.quaternion)
        T_base_openedobject = np.eye(4)
        T_base_openedobject[:3, :3] = R_robot.as_matrix()
        T_base_openedobject[:3, 3] = np.array(drawer_cabinet.absolute_location)

        T_openedobject_maxopenedhandle = np.eye(4)
        T_openedobject_maxopenedhandle[:3, 3] = np.array([0.0, center_to_handle + 0.05, 0.0]) # measured to center socket to handle

        T_base_maxopenedobjecthandle = np.matmul(T_base_openedobject, T_openedobject_maxopenedhandle)
        T_base_maxopenedobjecthandle_rotvec = R.from_rotvec(T_base_maxopenedobjecthandle[:3, :3])

        return T_base_maxopenedobjecthandle[:3, 3], T_base_maxopenedobjecthandle_rotvec.as_quat(canonical=False)

    def get_handle_pose(self, relevant_data): 
        p, q = self.target_opened_object_to_handle(relevant_data['target_opened_object'])
        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT
        p[2] += get_z_offset_from_center(relevant_data['target_opened_object'].name)

        ''' fix offset z rot '''
        offset_z_rot = 0.0
        q_ = np.array([1.,0.,0.,0.])
        q = UnitQuaternion([0.0,0.0,1.0,0.0]) # w,x,y,z
        q_2 = UnitQuaternion([q_[3], *q_[0:3]])

        rot = sm.SO3(q.R) * sm.SO3.Rz(q_2.rpy()[2]-np.pi/2+offset_z_rot)
        q = UnitQuaternion(rot).vec_xyzs

        return p, q
    
    def get_drawer_pose_at_state(self, relevant_data, drawer_state):
        if drawer_state == 'open':
            p, q = self.target_cabinet_to_max_opened_handle(relevant_data['target_drawer_cabinet'])
        elif drawer_state == 'close':
            p, q = self.target_cabinet_to_min_opened_handle(relevant_data['target_drawer_cabinet'])
        else: raise Exception("wrong drawer state")

        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT
        p[2] += get_z_offset_from_center(relevant_data['target_opened_object'].name)

        ''' fix offset z rot '''
        offset_z_rot = 0.0
        q_ = np.array([1.,0.,0.,0.])
        q = UnitQuaternion([0.0,0.0,1.0,0.0]) # w,x,y,z
        q_2 = UnitQuaternion([q_[3], *q_[0:3]])

        rot = sm.SO3(q.R) * sm.SO3.Rz(q_2.rpy()[2]-np.pi/2+offset_z_rot)
        q = UnitQuaternion(rot).vec_xyzs

        return p, q

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
            #     passprint(f"move_2: p: {p}; q: {q}")
            
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
            ''' Move opened! '''
            p, q = self.get_drawer_pose_at_state(relevant_data, drawer_state='open')
            
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
                return True, relevant_data
            else:
                return False, relevant_data

        return check_preconditions, self.get_ground_data, move_1, move_2, move_3, move_4, check_postconditions
