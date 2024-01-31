#!/usr/bin/env python3
from imitrob_templates.config import PutIntoTaskConfig
from imitrob_templates.templates import BaseTask

class PutIntoTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
        }
        super().__init__(task_config=PutIntoTaskConfig, modes=modes, *args, **kwargs)
        
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
        if (o.properties['reachable'] and
            o.properties['pickable'] and
            not o.properties['full-stack'] and
            not o.properties['glued'] and
            (s is None or ( # if condition on s given it is checked
                s.properties['reachable'] and
                s.properties['stackable'] and
                not s.properties['full-stack'] and
                s.is_type('container')
                ))
            ):
            ret = True
        else:
            ret = False
        
        assert super().is_feasible(o,s) == ret
        return ret