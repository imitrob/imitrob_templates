
from imitrob_templates.config import PushTaskConfig
from imitrob_templates.templates import BaseTask, TaskExecutionMode
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager


class PushTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 1
        modes = {
            TaskExecutionMode.BASIC: self.blueprint_mode_1,
            TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=PushTaskConfig, modes=modes, *args, **kwargs)

    def task_property_penalization_selections(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.0,
                'pickable':  1.0, 
                'stackable': 1.0,
                'pushable':  0.0, 
                'full-stack':1.0,
                'full-liquid':1.0,
                'glued':     0.0,
            }[property]

    def is_feasible(self, o, s=None):
        assert o is not None
        #assert s is None
        
        ret = None
        if (o.properties['reachable'] and
            o.properties['pushable'] and
            not o.properties['glued']
            ):
            ret = True
        else:
            ret = False
        
        assert super().is_feasible(o,s) == ret
        return ret