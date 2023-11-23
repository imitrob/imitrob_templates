#!/usr/bin/env python3
'''
from crow_nlp.nlp_crow.database.Ontology import Template
from crow_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector
from crow_nlp.nlp_crow.modules.ObjectGrounder import ObjectGrounder
from crow_nlp.nlp_crow.structures.tagging.TaggedText import TaggedText
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager
from crow_msgs.msg import CommandType
'''
import logging
'''
class PickTask(Template):
    """
    A template for the pick task = a robot instruction representing picking a specific object.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.lang = 'cs'
        # dialogue tools
        self.ui = UserInputManager(language = self.lang)
        # template detector - simplified view: seaches for a word that matches template
        self.templ_det = self.ui.load_file('templates_detection.json')
        self.parameters = ['action', 'action_type', 'target', 'target_type']
        self.target = [] # object to pick
        self.target_type = 'onto_uri'
        self.action_type = self.templ_det[self.lang]['pick']
        self.action = CommandType.PICK
        
        
        ###TODO: replaced by default robot behaviour (pick and home?)
        self.location = [0.53381, 0.18881, 0.22759]  # temporary "robot default" position
        self.location_type = 'xyz'
        
        self.logger = logging.getLogger(__name__)

        # related to parameters ?
        self.compare_types = ['action', 'selections']

    def match(self, tagged_text : TaggedText, language = 'en', client = None) -> None:
        # check all properties in object
        od = ObjectDetector(language = language, client = client)
        self.target = od.detect_object(tagged_text)

    def evaluate(self, language = 'en', client = None) -> None:
        # grounding
        self.lang = language
        self.ui = UserInputManager(language=self.lang)
        self.guidance_file = self.ui.load_file('guidance_dialogue.json')
        og = ObjectGrounder(language=self.lang, client=client)
        if self.target:
            self.target, self.target_ph_cls, self.target_ph_color, self.target_ph_loc = og.ground_object(obj_placeholder=self.target)
            names_to_add = ['target_ph_cls', 'target_ph_color', 'target_ph_loc']
            for name in names_to_add:
                if getattr(self, name):
                    self.parameters.append(name)

    def execute(self) -> None:
        self.target.location.x
        self.target.location.y
        print(self)

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False
'''
class PickTask():
    def __init__(self):
        self.name = 'pick'
        self.compare_types = ['target_action', 'target_object']
        self.complexity = 1

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
        return {'reachable': 0.4, # When object is not reachable, I still may want to  pick it, but the constraint action is penalized
                'pickable':  0.0, # When object is not pickable it cannot be picked at all
                'stackable': 1.0, 
                'pushable':  1.0, 
                'full-stack':1.0, #0.8, # When the object is full it can be still picked
                'full-liquid':1.0,
                'glued':     0.0, # When the object is glued it cannot be picked
            }[property]
    
    def is_feasible(self, o, s=None):
        #assert s is None
        assert o is not None

        if ( o.properties['reachable'] and  # When object is not reachable, I still may want to   pick it, but the constraint action is penalized
             o.properties['pickable'] and  # When object is not pickable it cannot be picked at all
             not o.properties['glued'] ):
            return True
        else:
            return False
        
    ''' There will be function that converts tagged text to HRICommand object '''
    @staticmethod
    def tagged_text_to_HRICommand(self, tagged_text):
        pass
    
    def match(self, tagged_text : TaggedText, language = 'en', client = None) -> None:
        ''' Checks if given command TaggedText corresponds to this template without checking the current detection
            Checks general classes in ontology (not in real world instances) 
        (Runs in NLP package and then in modality merger)
        
        Returns:
            match (Bool):
            parameters (?): W 
        '''
        # check all properties in object
        # od = ObjectDetector(language = language, client = client)
        # self.target = od.detect_object(tagged_text)
        
        # DRAFT:
        matched = False
        if matched:
            return True, ['pick', 'cup']
        else:
            return False, []
        
        
        
        
    def evaluate(self, language = 'en', client = None) -> None:
        ''' Grounding on the real objects 
        (Runs in Modality Merger)
        '''
        # grounding
        # self.lang = language
        # self.ui = UserInputManager(language=self.lang)
        # self.guidance_file = self.ui.load_file('guidance_dialogue.json')
        # og = ObjectGrounder(language=self.lang, client=client)
        # if self.target:
        #     self.target, self.target_ph_cls, self.target_ph_color, self.target_ph_loc = og.ground_object(obj_placeholder=self.target)
        #     names_to_add = ['target_ph_cls', 'target_ph_color', 'target_ph_loc']
        #     for name in names_to_add:
        #         if getattr(self, name):
        #             self.parameters.append(name)