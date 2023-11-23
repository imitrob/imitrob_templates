"""
class PlaceTask():
    def __init__(self):
        self.name = 'place'
        self.compare_types = ['template', 'storages']
        self.complexity = 1

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False

    def task_property_penalization(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.0,
                'pickable':  1.0, 
                'stackable': 0.0,
                'pushable':  1.0, 
                'full':      0.0,
                'glued':     1.0,
            }[property]

    def is_feasible(self, o=None, s=None):
        
        if s is None:
            return False
        
        if ( s.properties['reachable']
            ):
            return True
        else:
            return False
"""