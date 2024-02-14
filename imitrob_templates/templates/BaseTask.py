import enum, time
from typing import Any, Callable, Tuple
from imitrob_hri.imitrob_nlp.modules.ObjectGrounder import ObjectsGroundedData
from imitrob_hri.imitrob_nlp.modules.ObjectDetector import ObjectDetector
from imitrob_hri.imitrob_nlp.modules.ObjectGrounder import ObjectGrounder
from imitrob_hri.imitrob_nlp.modules.UserInputManager import UserInputManager
from imitrob_hri.imitrob_nlp.database.Ontology import Template
from imitrob_hri.merging_modalities.probs_vector import ProbsVector
from teleop_msgs.msg import Intent
from imitrob_hri.imitrob_nlp.nlp_utils import to_default_name
from copy import deepcopy

from imitrob_common_interfaces.msg import FrankaState

class TaskExecutionMode(enum.Enum):
    UNDEF = 0  # default value, should rise error
    BASIC = 1  # simple mode using basic robot actions
    MVAE = 2  # mode using MVAE to infer robot trajectory


class BaseTask(Template):

    def __init__(self, task_config: dict[str, Any], modes: dict[TaskExecutionMode, Callable], nlp=False):
        self.task_config = deepcopy(task_config)

        self.name = to_default_name(task_config['name'])
        self.task_prob = 1.0
        self.id = task_config['id']
        self.pars_compulsary = task_config['pars_compulsary']
        self.pars_voluntary = task_config['pars_voluntary']

        self.n_target_objects = 1

        self.target_object_penalizatiion = task_config['target_object_penalization']
        self.target_storage_penalization = task_config['target_storage_penalization']
        self.execution_config_params = task_config['execution_config_params']

        # Set placeholder for grounded variables
        # for parameter in self.parameters:
        #     setattr(self, parameter, '')

        self.scene = None

        self._modes = modes
        
        self.mm_pars_compulsary = task_config['mm_pars_compulsary']
        
        self._1_detected_data = {}
        ''' Result of match: Some semantic information about the matching '''

        self._2_grounded_data = {}
        ''' Result of ground: Real data about the grounding'''

        self.template_probs = ProbsVector(c='default')
        
        self.feasibility_requirements = task_config['requirements']

        # self.lang = 'cs'
        if nlp:
            self.ui = UserInputManager(language = self.lang)
            self.templ_det = self.ui.load_file('templates_detection.json')

    ''' Need to be done externally 
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
    '''
    def is_feasible(self, o, s=None):
        assert o is not None
        
        # If any Positive '+' requirement is not fulfilled, return False - not feasible 
        for req in self.feasibility_requirements['+']:
            if not o.properties[req]()[1]:
                return False
        # If any Negative '-' requirement is true, reutrn False - not feasible
        for req in self.feasibility_requirements['-']:
            if o.properties[req]()[1]:
                return False
        
        # All requirement are fulfilled
        return True

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

        
    def ground_scene(self, s):
        self.scene = s

    def task_property_penalization_target_objects(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return self.target_object_penalization[property]
    
    def task_property_penalization_target_objects(self, property):
        return self.target_storage_penalization[property]

    def execute(self, robot_client, ontology_client, mode=TaskExecutionMode.BASIC, **kwargs) -> Tuple[bool, str]:
        if mode not in self._modes:
            raise NotImplementedError(f"Mode {mode} is not implemented for task {self.name}!")
        mode_steps_generator = self._modes[mode]
        try:
            steps = mode_steps_generator(robot_client, ontology_client, **kwargs)
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
    

    # def match_intent(self, intent : Intent, scene) -> bool:
    #     ''' Same for all tasks
    #         Checks general classes in ontology (not in real world instances) 
    #     (Idea: Runs in NLP package and then in modality merger)
        
    #     Returns:
    #         match (Bool): 
    #     '''

    #     # is object on the scene?
    #     if scene.get_object_by_name(intent.target_object) is None:
    #         return False
    #     # target_action must be name of template
    #     if self.name != to_default_name(intent.target_action):
    #         return False

    #     ogdata = ObjectsGroundedData()
    #     ogdata.to = ProbsVector(intent.object_probs, intent.objects)
    #     self._2_grounded_data['to'] = ogdata

    #     return True
    
    def match_parsed_hricommand(self, hricommand, scene):

        ogdata = ObjectsGroundedData()
        ogdata.to = ProbsVector(hricommand['object_probs'], hricommand['objects'], c='default')
        self._2_grounded_data['to'] = ogdata

        if scene.get_object_by_name(self.target_object) is None:
            return False
        # target_action must be name of template
        if self.name != to_default_name(self.target_action):
            return False
        
        print("_2_grounded_data",self._2_grounded_data)

        return True        

    
    ''' MM compatibility '''
    def has_compare_type(self, compare_type):
        if compare_type in self.mm_pars_compulsary:
            return True
        else:
            return False
        

    def grounded_data_filled(self):
        """Compulsary data filled

        Returns:
            Bool: Ready to execute template
        """        
        for par in self.pars_compulsary:
            if getattr(self, par) is None:
                print(f"grounded_data_filled parameter: {par} not filled")
                return False
        return True
    
    @staticmethod
    def nlp_match(tagged_text : Intent, language = 'en', client = None) -> bool:
        ''' Checks if given command TaggedText corresponds to this template without checking the current detection
            Checks general classes in ontology (not in real world instances) 
        
        Returns:
            match (Bool): 
        
        Current running pipeline:        
        process_sentence_callback()
        └── process_text()
            └── process_node()
                └── nlp_match()
        '''
        # requirements = deepcopy(self.parameters) # pars_compulsary + pars_optional 
        _1_detected_data = {}

        all_requirements = ['target_action', 'target_object', 'target_storage']
        for req in all_requirements:


            if req == 'target_action': continue

            if req == 'target_object':
                od = ObjectDetector(language = language, client = client)
                objs_det = od.detect_object(tagged_text, detect_type='target_object')
                
                if objs_det is not None :
                    _1_detected_data['to'] = deepcopy(objs_det)
                    print(f" ******* [Base Task] nlp_match:TO ended with: *******\n{objs_det.objs_mentioned_cls}\n************************************")
                else:
                    # Detect penalized properties
                    _1_detected_data['to'] = od.detect_standalone_properties(tagged_text)
                    print(f" ******* [Base Task] nlp_match:TO ended with: None ******* ")

            if req == 'target_storage':
                od = ObjectDetector(language = language, client = client)
                stgs_det = od.detect_object(tagged_text, detect_type='target_storage')
                
                # od = LocationDetector(language = language, client = client)
                # stgs_det = od.detect_storage(tagged_text)

                if stgs_det is not None:
                    print(f" ******* [Base Task] nlp_match:TS ended with: *******\n{stgs_det.objs_mentioned_cls}\n************************************")
                    _1_detected_data['ts'] = deepcopy(stgs_det)
                else:
                    print(f" ******* [Base Task] nlp_match:TS ended with: None ******* ")
            
            # raise Exception(f"requirement: {req} is not in [target_action, target_object, target_storage]!")
        return _1_detected_data

    @staticmethod
    def nlp_ground(_1_detected_data, language = 'en', client = None):
        ''' Grounding on the real objects 
        (Runs in Modality Merger)

        Current running pipeline:        
        process_sentence_callback()
        └── process_text()
            └── process_node()
                └── nlp_match()
                └── nlp_ground()
        '''
        # self.lang = language
        # self.ui = UserInputManager(language=self.lang)
        # self.guidance_file = self.ui.load_file('guidance_dialogue.json')
        og = ObjectGrounder(language=language, client=client)
        _2_grounded_data = {}

        if 'to' in _1_detected_data.keys():
            objs_grd = og.ground_object(obj_placeholder = _1_detected_data['to'])
            if objs_grd is not None:
                _2_grounded_data['to'] = deepcopy(objs_grd)

        if 'ts' in _1_detected_data.keys():
            stgs_grd = og.ground_object(obj_placeholder = _1_detected_data['ts'])
            if stgs_grd is not None:
                _2_grounded_data['ts'] = deepcopy(stgs_grd)
        
        return _2_grounded_data
    
    ''' Property readers concept 
        Data are saved in grounded data
        There are names, probs, etc.. where it becommes unreadable
        Here, for every parameter, the target is picked
    '''
    @property
    def target_object(self):
        ''' Concept '''
        if 'to' in self._2_grounded_data:
            return self._2_grounded_data['to'].to.max
        else:
            return None

    @property
    def target_storage(self):
        ''' Concept '''
        if 'ts' in self._2_grounded_data:
            return self._2_grounded_data['ts'].to.max
        else:
            return None

    def blueprint_mode_1(self):
        raise NotImplementedError("This should be overloaded")
    
    def mvae_mode(self):
        raise NotImplementedError("This should be overloaded")