class PourTask():
    def __init__(self):
        self.name = 'pour'
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
        return {'reachable': 0.0,
                'pickable':  0.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':0.2,
                'full-liquid':0.2,
                'glued':     0.0,
            }[property]

    def task_property_penalization_storages(self, property):
        ''' How much to penalize for given property - weighted
            Set up using common sense
            e.g. when object is not reachable, how much it matters for pick-task -> quite significant
        '''
        return {'reachable': 0.0,
                'pickable':  1.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full-stack':0.2,
                'full-liquid':0.2,
                'glued':     1.0,
            }[property]


    def is_feasible(self, o=None, s=None):
        assert o is not None
        #assert s is not None
        
        if (o.properties['reachable'] and
            o.properties['pickable'] and
            not o.properties['full-stack'] and
            o.properties['full-liquid'] and
            not o.properties['glued'] and 
            (s is None or ( # if condition on s given it is checked
                s.properties['reachable'] and
                not s.properties['full-stack'] and 
                not s.properties['full-liquid'] and
                s.is_type('liquid-container')
                ))
            ):
            return True
        else:
            return False