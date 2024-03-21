#!/usr/bin/env python3
import logging
  
from imitrob_templates.config import StackTaskConfig, TEMPORARY_VISION_ERROR_CORRECTION_POINT
from imitrob_templates.templates import BaseTask
from imitrob_templates.object_config import get_z_offset_from_center
from imitrob_templates.utils import get_quaternion_eef


from copy import deepcopy
import numpy as np
from typing import Dict, Any, Tuple

  
class StackTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
        }
        super().__init__(task_config=StackTaskConfig, modes=modes, *args, **kwargs)
        
    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        if tagged_text.contains_text(templ_det[lang]['stack']):
            return True, 'stack'
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
                'full-stack':0.2,
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
                not s.properties['full-stack']()[1]
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
        # Here is listed all the data parameters to complete
        # the Task, e.g. Here single object_name
        target_object_name: str = self.target_object
        target_storage_name: str = self.target_storage

        # Load target_object using its ID
        relevant_data: Dict[str, Any] = {
            'target_object': self.scene.get_object_by_name(target_object_name),
            'target_storage': self.scene.get_object_by_name(target_storage_name),
        }

        # Check once again that taget_object and taget_storage are detected
        assert relevant_data['target_object'] is not None
        assert relevant_data['target_storage'] is not None

        # Returns list of grounded data
        return True, relevant_data

    def get_mode_1_move_1(self, relevant_data):
        target_object = relevant_data['target_object']
        ''' Move above the picking object '''
        p = deepcopy(np.array(target_object.absolute_location))

        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT

        p[2] += self.execution_config_params['move_near_z_offset']
        # Get Robot EEF rotation from object orientation
        # q = deepcopy(np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN)))
        q = [1.0,0.0,0.0,0.0]

        #     :move_gripper, move_robot, p, q, gripper
        return True        , True,       p, q, 'open'

    def get_mode_1_move_2(self, relevant_data):
        target_object = relevant_data['target_object']
        ''' Move to object grasp point '''
        p = deepcopy(np.array(target_object.absolute_location))
        p[2] += get_z_offset_from_center(target_object.name)
        # q = deepcopy(np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN)))
        q = [1.0,0.0,0.0,0.0]

        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT

        #     :move_gripper, move_robot, p, q, gripper
        return True        , True,       p, q, 'close'


    def get_mode_1_move_3(self, relevant_data):
        target_object = relevant_data['target_object']
        ''' Move to object grasp point '''
        p = deepcopy(np.array(target_object.absolute_location))
        p[2] += get_z_offset_from_center(target_object.name)
        # q = deepcopy(np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN)))
        q = [1.0,0.0,0.0,0.0]

        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT
        ''' Move up littlebit '''
        p[2] += self.execution_config_params['move_final_z_offset']

        #     :move_gripper, move_robot, p, q, gripper
        return False       , True,       p, q, ''

    def get_mode_1_move_4(self, relevant_data):
        target_storage = relevant_data['target_storage']
        ''' Move above the placing object '''
        p = deepcopy(np.array(target_storage.absolute_location))

        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT

        p[2] += self.execution_config_params['placing_object_move_real_z_offset']
        
        # Get Robot EEF rotation from object orientation
        #q = np.array(get_quaternion_eef(target_object.quaternion, target_object.nlp_name_EN))
        q = np.array([1.,0.,0.,0.])

        #     :move_gripper, move_robot, p, q, gripper
        return False        , True,       p, q, ''

    def get_mode_1_move_5(self, relevant_data):
        target_storage = relevant_data['target_storage']
        ''' Move to object grasp point '''
        p = deepcopy(np.array(target_storage.absolute_location))
        p[2] += (get_z_offset_from_center(target_storage.name) + self.execution_config_params['placing_object_move_final_z_offset'])
        
        p += TEMPORARY_VISION_ERROR_CORRECTION_POINT
        q = [1.0, 0.0, 0.0, 0.0]

        #     :move_gripper, move_robot, p, q, gripper
        return True        , True,       p, q, 'open'


    def blueprint_mode_1(self, robot_client, ontology_client):
        '''
        Involving self vars:
            object_names (String[] or Int[]): Unique IDs of objects

        '''

        def check_preconditions(relevant_data):
            # if not check_grasped(): return False
            return True, relevant_data

        def move_1(relevant_data):
            move_gripper, move_robot, p, q, gripper = self.get_mode_1_move_1(relevant_data)
            self.pqg_execute(move_gripper, move_robot, p, q, gripper, robot_client)

            return True, relevant_data

        def move_2(relevant_data):
            move_gripper, move_robot, p, q, gripper = self.get_mode_1_move_2(relevant_data)
            self.pqg_execute(move_gripper, move_robot, p, q, gripper, robot_client)

            # Pragmatic: Pick target_object should have included some time delay
            # time.sleep(2.)
            relevant_data['p'], relevant_data['q'] = p, q
            return True, relevant_data

        def move_3(relevant_data):
            move_gripper, move_robot, p, q, gripper = self.get_mode_1_move_3(relevant_data)
            self.pqg_execute(move_gripper, move_robot, p, q, gripper, robot_client)
            
            return True, relevant_data

        def move_4(relevant_data):
            move_gripper, move_robot, p, q, gripper = self.get_mode_1_move_4(relevant_data)
            self.pqg_execute(move_gripper, move_robot, p, q, gripper, robot_client)
            
            return True, relevant_data

        def move_5(relevant_data):
            move_gripper, move_robot, p, q, gripper = self.get_mode_1_move_5(relevant_data)
            self.pqg_execute(move_gripper, move_robot, p, q, gripper, robot_client)
            
            return True, relevant_data

        def check_postconditions(relevant_data):

            # if self.check_grasped(): return False

            return True, relevant_data

        return check_preconditions, self.get_ground_data, move_1, move_2, move_3, move_4, move_5, check_postconditions
