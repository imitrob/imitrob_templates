
from imitrob_templates.templates import BaseTask
from imitrob_templates.config import PourTaskConfig

class PourTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
        }
        super().__init__(task_config=PourTaskConfig, modes=modes, *args, **kwargs)
        
    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        if tagged_text.contains_text(templ_det[lang]['pour']):
            return True, 'pour'
        else:
            return False, ''

    def task_property_penalization_selections(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.0,
                'pickable':  0.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':0.2,
                'full-liquid':0.2,
                'glued':     0.0,
            }[property]

    def task_property_penalization_storages(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.0,
                'pickable':  1.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':0.2,
                'full-liquid':0.2,
                'glued':     1.0,
            }[property]


    def is_feasible(self, o=None, s=None):
        assert o is not None
        #assert s is not None
        
        ret = None
        # if (o.properties['reachable']()[1] and
        #     o.properties['pickable']()[1] and
        #     not o.properties['full-stack']()[1] and
        #     o.properties['full-liquid']()[1] and
        #     not o.properties['glued']()[1] and 
        #     (s is None or ( # if condition on s given it is checked
        #         s.properties['reachable']()[1] and
        #         not s.properties['full-stack']()[1] and 
        #         not s.properties['full-liquid']()[1] and
        #         s.is_type('liquid-container')
        #         ))
        #     ):
        #     ret = True
        if (o.properties['reachable']()[1] and
            o.properties['pickable']()[1] and
            not o.properties['full-stack']()[1] and
            o.properties['full-liquid']()[1] and
            not o.properties['glued']()[1] and 
            (s is None or ( # if condition on s given it is checked
                s.properties['reachable']()[1] and
                not s.properties['full-stack']()[1] and 
                not s.properties['full-liquid']()[1] and
                not s.properties['full-container']()[1] and
                s.is_type('liquid-container')
                ))
            ):
            ret = True
        else:
            ret = False
        
        assert super().is_feasible(o,s) == ret
        return ret