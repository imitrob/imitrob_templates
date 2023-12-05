#!/usr/bin/env python3
'''
from crow_nlp.nlp_crow.database.Ontology import Template
from crow_nlp.nlp_crow.modules.LocationDetector import LocationDetector

#from crow_nlp.nlp_crow.modules.LocationGrounder import LocationGrounder
from crow_nlp.nlp_crow.modules.ObjectDetector import ObjectDetector
from crow_nlp.nlp_crow.modules.ObjectGrounder import ObjectGrounder
from crow_nlp.nlp_crow.structures.tagging.TaggedText import TaggedText
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager
'''
import logging

'''
class PutTask(Template):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        #self.register_parameter(name="object_to_put", value=self.onto.Object)
        #self.register_parameter(name="location", value=self.onto.Location)
        self.logger = logging.getLogger(__name__)
        self.ui = UserInputManager()

        self.templ_det = self.ui.load_file('templates_detection.json')

        # related to parameters ?
        self.compare_types = ['action', 'selections']

    def match(self, tagged_text : TaggedText, language = 'en') -> None:
        od = ObjectDetector(language = language)
        ld = LocationDetector(language = language)

        self.location = ld.detect_location(tagged_text)
        try:
            put_index = tagged_text.indices_of(self.templ_det[language]['put'])[0]
            tagged_text_cut = tagged_text.cut(put_index + 1, put_index + 4)

            self.object_to_put = od.detect_object(tagged_text_cut)
        except:
            self.logger.debug(f"Put index not detected.")

    def evaluate(self, language = 'en') -> None:
        # check if the object to be put down is in the workspace
        self.lang = language
        og = ObjectGrounder(language = self.lang)
        self.object_to_put = og.ground_object(obj_placeholder=self.object_to_put)

        if self.onto.RelativeLocation in self.location.is_instance_of:
            raise NotImplementedError()
            #lg = LocationGrounder()
            #self.location = lg.ground_location(self.location)

    def has_compare_type(self, compare_type):
        return False


    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False
'''    
class PutIntoTask():
    def __init__(self):
        self.name = 'put-into'
        self.compare_types = ['template', 'selections', 'storages']
        self.complexity = 2

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
            return True
        else:
            return False