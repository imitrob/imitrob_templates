import enum
from typing import Any, Callable, Tuple

from crow_nlp.nlp_crow.database.Ontology import Template


class TaskExecutionMode(enum.Enum):
    UNDEF = 0  # default value, should rise error
    BASIC = 1  # simple mode using basic robot actions
    MVAE = 2  # mode using MVAE to infer robot trajectory


class BaseTask(Template):

    def __init__(self, task_config: dict[str, Any], modes: dict[TaskExecutionMode, Callable]):
        self.task_config = task_config

        self.name = task_config['name']
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
