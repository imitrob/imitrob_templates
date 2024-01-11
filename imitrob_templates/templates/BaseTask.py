import enum, time
from typing import Any, Callable, Tuple

from crow_nlp.nlp_crow.database.Ontology import Template
from teleop_msgs.msg import Intent
from imitrob_hri.imitrob_nlp.nlp_utils import to_default_name

from imitrob_common_interfaces.msg import FrankaState

class TaskExecutionMode(enum.Enum):
    UNDEF = 0  # default value, should rise error
    BASIC = 1  # simple mode using basic robot actions
    MVAE = 2  # mode using MVAE to infer robot trajectory


class BaseTask(Template):

    def __init__(self, task_config: dict[str, Any], modes: dict[TaskExecutionMode, Callable]):
        self.task_config = task_config

        self.name = to_default_name(task_config['name'])
        self.id = task_config['id']
        self.pars_compulsary = task_config['pars_compulsary']
        self.pars_voluntary = task_config['pars_voluntary']

        self.n_target_objects = 1

        self.target_object_penalizatiion = task_config['target_object_penalization']
        self.execution_config_params = task_config['execution_config_params']

        # Set placeholder for grounded variables
        for parameter in self.parameters:
            setattr(self, parameter, '')

        self.scene = None

        self._modes = modes

        self.create_subscribtion(FrankaState, '/robot_state', self.franka_state_clb, 5)
        self.get_last_state_flag = False
        self.last_state = None

    def check_grasped(self):            
        # wait for the lastest franka state
        self.get_last_state_flag = True
        i = 0
        while self.last_state is None:
            i += 1
            time.sleep(0.05)
            if i > 1000:
                print("Franka state not received!")

        return self.last_state['grasped'] # check 'grasped' is the key

    def franka_state_clb(self, msg):
        if self.get_last_state_flag:
            self.last_state = msg
            self.get_last_state_flag = False

    def get_all_filled_voluntary_parameters(self):
        # TODO:
        return self.pars_voluntary

    @property
    def parameters(self):
        """ The Grounded variables

        Returns:
            List of Strings: pars compulsary & pars voluntary
        """
        params = self.pars_compulsary
        params.extend(self.get_all_filled_voluntary_parameters())
        return params

    @property
    def compare_types(self):
        return self.pars_compulsary

    @property
    def complexity(self):
        return self.compare_types - 1  # complexity = 1

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False
        
    def ground_realpositions(self, s):
        self.scene = s

    def task_property_penalization_target_objects(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return self.target_object_penalization[property]

    def execute(self, robot_client, mode=TaskExecutionMode.BASIC, **kwargs) -> Tuple[bool, str]:
        if mode not in self._modes:
            raise NotImplementedError(f"Mode {mode} is not implemented for task {self.name}!")
        mode_steps_generator = self._modes[mode]
        try:
            steps = mode_steps_generator(robot_client, **kwargs)
        except BaseException as e:
            return False, f"Error generating steps for task {self.name}!\n{e}"

        relevant_data = {}
        for i, step in enumerate(steps, start=1):
            print(f"Running {str(step)}")
            try:
                ret, relevant_data = step(relevant_data)
            except ValueError as e:
                emsg = f"ValueError when executing step {i}/{len(steps)} of task {self.name}!"
                emsg += f"\nMaybe 'relevant_data' does not contain all necessary data?\n{e}"
                return False, emsg
            except BaseException as e:
                return False, f"Error executing step {i}/{len(steps)} of task {self.name}!\n{e}"

            if ret:
                print(f"Step {i}/{len(steps)} done.")
            else:
                return False, f"Step {i}/{len(steps)} of task {self.name} returned False!"

        return True, "Success"

    @property
    def target_action(self):
        ''' alias for self.name: target_action is name of template '''
        return self.name
    
    @target_action.setter
    def target_action(self, name):
        ''' target_action is self.name: name of this action '''
        if name == '':
            return
        elif name != self.name:
            raise Exception("Setting target_action is not same as created action template")
        else:
            return
    

    def match_intent(self, intent : Intent, scene) -> bool:
        ''' Same for all tasks
            Checks general classes in ontology (not in real world instances) 
        (Idea: Runs in NLP package and then in modality merger)
        
        Returns:
            match (Bool): 
        '''

        # is object on the scene?
        if scene.get_object_by_name(intent.target_object) is None:
            return False
        # target_action must be name of template
        if self.name != to_default_name(intent.target_action):
            return False

        self.target_object = intent.target_object

        return True
    