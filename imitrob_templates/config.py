

PickTask = {
    'name': 'pick',
    'pars_compulsary': ['target_action', 'target_object'],
    'pars_voluntary': [],
    # inverted soft-requirements
    'target_object_penalization': {
        'reachable': 0.4, # When object is not reachable, I still may want to  pick it, but the constraint action is penalized
        'pickable':  0.0, # When object is not pickable it cannot be picked at all
        'stackable': 1.0, 
        'pushable':  1.0, 
        'full-stack':1.0, #0.8, # When the object is full it can be still picked
        'full-liquid':1.0,
        'glued':     0.0, # When the object is glued it cannot be picked
        },
    # hard requirements
    'requirements': {'+': ['reachable', 'pickable'], '-': ['glued']},
    'complexity': 1
}

