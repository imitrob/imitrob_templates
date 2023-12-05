class MoveUpTask():
    def __init__(self):
        self.name = 'move-up'
        self.compare_types = ['template']
        self.complexity = 0

    def has_compare_type(self, compare_type):
        if compare_type in self.compare_types:
            return True
        else:
            return False
        
    def task_property_penalization(self, property):
        raise Exception("Should not have properties")
        return {'reachable': 1.0,
                'pickable':  1.0, 
                'stackable': 1.0,
                'pushable':  1.0, 
                'full':      1.0,
                'glued':     1.0,
            }[property]

    def is_feasible(self, o=None, s=None):
        #assert o is None
        #assert s is None

        return True