#!/usr/bin/env python3
'''
from crow_nlp.nlp_crow.database.Ontology import Template

from crow_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector
from crow_nlp.nlp_crow.modules.ObjectGrounder import ObjectGrounder
from crow_nlp.nlp_crow.modules.PositionStorageGrounder import PositionStorageGrounder
from crow_nlp.nlp_crow.modules.PositionStorageDetector import PositionStorageDetector
from crow_nlp.nlp_crow.structures.tagging.TaggedText import TaggedText
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager
from crow_msgs.msg import CommandType
'''
import logging
'''
class PointTask(Template):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lang = 'cs'
        self.ui = UserInputManager(language = self.lang)
        self.templ_det = self.ui.load_file('templates_detection.json')
        self.parameters = ['action', 'action_type', 'target_type']
        self.target = [] # object to point
        self.position = None
        self.storage = None
        self.target_type = 'onto_uri'
        self.action_type = self.templ_det[self.lang]['point']
        self.action = CommandType.POINT
        self.logger = logging.getLogger(__name__)

        # related to parameters ?
        self.compare_types = ['action', 'selections']

    def match(self, tagged_text : TaggedText, language = 'en', client = None) -> None:
        psd = PositionStorageDetector(language=language, client = client)
        self.position = psd.detect_position(tagged_text)
        self.storage = psd.detect_storage(tagged_text)
        if self.position:
            self.parameters.append('position')
        elif self.storage:
            self.parameters.append('storage')
        else:
            od = ObjectDetector(language = language, client = client)
            self.target = od.detect_object(tagged_text)
            self.parameters.append('target')

    def evaluate(self, language = 'en', client = None) -> None:
        # check if the object to be put down is in the workspace
        self.lang = language
        self.ui = UserInputManager(language = self.lang)
        self.guidance_file = self.ui.load_file('guidance_dialogue.json')
        og = ObjectGrounder(language = self.lang, client = client)
        if self.target:
            self.target, self.target_ph_cls, self.target_ph_color, self.target_ph_loc = og.ground_object(obj_placeholder=self.target)
            names_to_add = ['target_ph_cls', 'target_ph_color', 'target_ph_loc']
            for name in names_to_add:
                if getattr(self, name):
                    self.parameters.append(name)
        elif self.position:
            pg = PositionStorageGrounder(language = self.lang, client = client)
            self.target = pg.ground_position(self.position)
            self.parameters.append('target')
        elif self.storage:
            pg = PositionStorageGrounder(language = self.lang, client = client)
            self.target = pg.ground_storage(self.storage)
            self.parameters.append('target')

    def has_compare_type(self, compare_type):
        return False


    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False
'''
class PointTask():
    def __init__(self):
        self.name = 'point'
        self.compare_types = ['template', 'selections']
        self.complexity = 1

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False

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
                'glued':     1.0,
            }[property]

    def is_feasible(self, o=None, s=None):
        assert o is not None 
        #assert s is None

        return True
