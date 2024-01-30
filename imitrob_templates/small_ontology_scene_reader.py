
import rclpy
from rclpy.node import Node

from crow_ontology.crowracle_client import CrowtologyClient

import numpy as np
from context_based_gesture_operation.srcmodules.Scenes import Scene as Scene2
from context_based_gesture_operation.srcmodules.Objects import Object as Object2
from context_based_gesture_operation.srcmodules.Objects import CrowObject2

from imitrob_hri.data.scene3_def import Object3, Scene3
from imitrob_templates.object_config import get_static_properties

class SceneOntologyClient():
    def __init__(self, rosnode):
        self.crowracle = CrowtologyClient(node=rosnode)
        self.onto = self.crowracle.onto
        
        print("Ontology Client Ready")
        
    
    def add_dummy_cube(self):
        o_list = self.get_objects_from_onto()
        if len(o_list) == 0:
           print("Adding a dummy cube into ontology")
           self.crowracle.add_test_object("cube_holes")    
    
    def get_objects_from_onto(self):
        o_list = self.crowracle.getTangibleObjectsProps()
        #print("Onto objects:")
        #print(o_list)
        return o_list
    
    @staticmethod
    def mocked_update_scene():
        s = None
        s = Scene2(init='object', random=False)
        return s

    NAME2TYPE = {
        'cube': 'Object',
        
    }

    def get_scene2(self):
        s = None
        s = Scene2(init='', objects=[], random=False)
        
        objects = self.crowracle.getTangibleObjectsProps()
        ''' object list; item is dictionary containing uri, id, color, color_nlp_name_CZ, EN, nlp_name_CZ, nlp_name_EN; absolute_location'''

        '''
        import rdflib
        uri = rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551')
        '''
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]

        # Colors:
        # [COLOR_GREEN. 

        for object in objects:
            ''' o is dictionary containing properties '''
            uri = object['uri']
            id = object['id']

            # created name `wheel_od_0` extracted from 'http://imitrob.ciirc.cvut.cz/ontologies/crow#wheel_od_0'
            name = str(object['uri']).split("#")[-1]

            color = object['color']
            color_nlp_name_CZ = object['color_nlp_name_CZ']
            color_nlp_name_EN = object['color_nlp_name_EN']
            nlp_name_CZ = object['nlp_name_CZ']
            nlp_name_EN = object['nlp_name_EN']
            absolute_location = object['absolute_location']
            absolute_orientation = object['absolute_quaternion']
            
            o = CrowObject2(name=name, type=name.split('_od_')[0], position_real=np.array(absolute_location), orientation=absolute_orientation, random=False)
            # o.quaternion = np.array(object['pose'][1])
            # o.color_uri = color
            o.color = color_nlp_name_EN
            # o.color_name_CZ = color_nlp_name_CZ
            
            o.nlp_name_CZ = nlp_name_CZ
            o.nlp_name_EN = nlp_name_EN
            o.crow_id = id
            o.crow_uri = uri
            
            s.objects.append(o)

        return s
    
    
    
    
    def get_scene3(self):
        
        objects = self.crowracle.getTangibleObjectsProps()
        
        # testing if object timestamp changes 
        # import time
        # while True:
        #     time.sleep(0.5)
        #     res = self.onto.query(self.crowracle._query_check_time_enable_disable)
        #     print([(g["obj"], g["stamp"], g["enabled"].toPython()) for g in res])

        print(objects)
        ''' object list; item is dictionary containing uri, id, color, color_nlp_name_CZ, EN, nlp_name_CZ, nlp_name_EN; absolute_location'''

        '''
        import rdflib
        uri = rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551')
        '''
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]

        # Colors:
        # [COLOR_GREEN.
        
        scene_objects = []
        for object in objects:
            ''' o is dictionary containing properties '''
            uri = object['uri']
            id = object['id']

            # created name `wheel_od_0` extracted from 'http://imitrob.ciirc.cvut.cz/ontologies/crow#wheel_od_0'
            name = str(object['uri']).split("#")[-1]

            color = object['color']
            color_nlp_name_CZ = object['color_nlp_name_CZ']
            color_nlp_name_EN = object['color_nlp_name_EN']
            nlp_name_CZ = object['nlp_name_CZ']
            nlp_name_EN = object['nlp_name_EN']
            absolute_location = object['absolute_location']
            
            # get_the_uri and from it the size, roundness-top, weight, contains, types are given
            
            properties = get_static_properties(name)

            observations = {
                'name': name,
                'size': properties['size'], # [m]
                'position': absolute_location, # [m,m,m]
                'roundness-top': properties['roundness-top'], # [normalized belief rate]
                'weight': properties['weight'], # [kg]
                'contains': properties['contains'], # normalized rate being full 
                'contain_item': properties['contain_item'], # how many items contains
                'types': properties['types'],
                'glued': properties['glued'],
            }
            
            o = Object3(observations)
            # o.quaternion = np.array(object['pose'][1])
            # o.color_uri = color
            # o.color = color_nlp_name_EN
            # o.color_name_CZ = color_nlp_name_CZ
            
            # o.nlp_name_CZ = nlp_name_CZ
            # o.nlp_name_EN = nlp_name_EN
            # o.crow_id = id
            # o.crow_uri = uri
            
            scene_objects.append(o)
        
        s = None
        storages = []
        template_names = ['pick up', 'point']
        s = Scene3(scene_objects, storages, template_names)
        
        return s        

if __name__ == '__main__':
    rclpy.init()
    rosnode = Node('ontology_reader_node')
    soc = SceneOntologyClient(rosnode)
    s2 = soc.get_scene2()
    print("Scene 2")
    print(f"{s2}")
    s3 = soc.get_scene3()
    print("Scene 3")
    print(f"{s3}")
    print("Done")
    rclpy.spin(rosnode)
    
    