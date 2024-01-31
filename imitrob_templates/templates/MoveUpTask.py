
from imitrob_templates.templates.BaseTask import BaseTask
from imitrob_templates.config import MoveUpTaskConfig

class MoveUpTask(BaseTask):
    def __init__(self, *args, **kwargs):
        self.n_target_objects = 0
        modes = {
            # TaskExecutionMode.BASIC: self.blueprint_mode_1,
            # TaskExecutionMode.MVAE: self.mvae_mode
        }
        super().__init__(task_config=MoveUpTaskConfig, modes=modes, *args, **kwargs)
    
    @staticmethod
    def detect_fun(tagged_text, templ_det, lang):
        
        if tagged_text.contains_text(templ_det[lang]['move up']):
            return True, 'move-up'
        else:
            return False, ''


    def task_property_penalization(self, property):
        raise Exception("Should not have properties")

    def is_feasible(self, o=None, s=None):
        
        #assert o is None
        #assert s is None

        assert super().is_feasible(o,s) == True
        return True
