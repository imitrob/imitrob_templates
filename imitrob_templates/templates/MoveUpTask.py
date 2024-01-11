
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
    
    def task_property_penalization(self, property):
        raise Exception("Should not have properties")

    def is_feasible(self, o=None, s=None):
        #assert o is None
        #assert s is None

        return True
