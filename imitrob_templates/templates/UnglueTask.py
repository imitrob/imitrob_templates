#!/usr/bin/env python3

import logging
from imitrob_templates.config import UnglueTaskConfig
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager

class UnglueTask(BaseTask):
    def __init__(self, nlp=True, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=UnglueTaskConfig, modes=modes, *args, **kwargs)
        
        # might be deleted if not needed 
        if nlp:
            self.lang = 'cs'
            self.ui = UserInputManager(language = self.lang)
            self.templ_det = self.ui.load_file('templates_detection.json')
            self.parameters = ['action', 'action_type', 'target', 'target_type']
            # self.target_object = [] #object to pick
        
        # self.target_type = 'onto_uri'
        # self.action_type = self.templ_det[self.lang]['unglue']
        self.target_action = 'unglue'


    def has_compare_type(self, compare_type):
        if compare_type in self.mm_pars_compulsary:
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
                'glued':     0.0,
            }[property]

    def is_feasible(self, o=None, s=None):
        #assert s is None
        assert o is not None

        if ( o.properties['glued'] ):
            return True
        else:
            return False
        
    def blueprint_mode_1(self):
        raise Exception()
    
    def mvae_mode(self):
        raise Exception()