#!/usr/bin/env python3
from imitrob_templates.config import PutIntoTaskConfig
from imitrob_templates.templates import BaseTask, TaskExecutionMode

from typing import Dict, Any, Tuple

from copy import deepcopy
import numpy as np
from imitrob_templates.config import TEMPORARY_VISION_ERROR_CORRECTION_POINT
from spatialmath import UnitQuaternion
import spatialmath as sm
from imitrob_templates.object_config import get_z_offset_from_center

class PutIntoTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
        }
        super().__init__(task_config=PutIntoTaskConfig, modes=modes, *args, **kwargs)
        
    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        if tagged_text.contains_text(templ_det[lang]['put']):
            return True, 'put-into'
        else:
            return False, ''

    def task_property_penalization_selections(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.3,
                'pickable':  0.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':0.0,
                'full-liquid':1.0,
                'glued':     0.0,
            }[property]
    
    def task_property_penalization_storages(self, property):
        return {'reachable': 0.0,
                'pickable':  1.0, 
                'stackable': 0.0,
                'pushable':  1.0, 
                'full-stack':0.0,
                'full-liquid':1.0,
                'glued':     1.0,
            }[property]


    def is_feasible(self, o, s=None):
        assert o is not None
        #assert s is not None

        ret = None
        if (o.properties['reachable']()[1] and
            o.properties['pickable']()[1] and
            not o.properties['full-stack']()[1] and
            not o.properties['glued']()[1] and
            (s is None or ( # if condition on s given it is checked
                s.properties['reachable']()[1] and
                s.properties['stackable']()[1] and
                not s.properties['full-stack']()[1] and
                s.is_type('container')
                ))
            ):
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
        target_object_name: str = self.target_object

        # Load target_object using its ID
        relevant_data: Dict[str, Any] = {
            'target_object': self.scene.get_object_by_name(target_object_name),
        }

        # Check once again that taget_object is detected
        assert relevant_data['target_object'] is not None

        # Returns list of grounded data
        return True, relevant_data

    def blueprint_mode_1(self, robot_client, ontology_client):
        '''
        Involving self vars:
            object_names (String[] or Int[]): Unique IDs of objects

        '''

        def check_preconditions(relevant_data):
            # if not check_grasped(): return False
            return True, relevant_data

        def move_1(relevant_data):
            ''' Move above the picking object '''
            p = deepcopy(np.array(relevant_data['target_object'].absolute_location))

            p += TEMPORARY_VISION_ERROR_CORRECTION_POINT
            p[2] += self.execution_config_params['move_near_z_offset']
            p[2] += get_z_offset_from_center(relevant_data['target_object'].name)

            ''' fix offset z rot '''
            offset_z_rot = 0.0
            q_ = np.array([1.,0.,0.,0.])
            q = UnitQuaternion([0.0,0.0,1.0,0.0]) # w,x,y,z
            q_2 = UnitQuaternion([q_[3], *q_[0:3]])

            rot = sm.SO3(q.R) * sm.SO3.Rz(q_2.rpy()[2]-np.pi/2+offset_z_rot)
            q = UnitQuaternion(rot).vec_xyzs

            robot_client.move_pose(p, q)
            robot_client.open_gripper()

            return True, relevant_data

        def move_3(relevant_data):
            # p, q = relevant_data['p'], relevant_data['q']
            # ''' Move up littlebit '''
            # p[2] += self.execution_config_params['move_final_z_offset']

            # robot_client.move_pose(p, q)
            

            return True, relevant_data

        def check_postconditions(relevant_data):

            # if self.check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, check_postconditions
