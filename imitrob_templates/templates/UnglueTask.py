#!/usr/bin/env python3

import logging
from imitrob_templates.config import UnglueTaskConfig
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager

class UnglueTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=UnglueTaskConfig, modes=modes, *args, **kwargs)

    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        if tagged_text.contains_text(templ_det[lang]['unglue']):
            return True, 'unglue'
        else:
            return False, ''

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

        ret = None
        if ( o.properties['glued'] ):
            ret = True
        else:
            ret = False

        assert super().is_feasible(o,s) == ret
        return ret