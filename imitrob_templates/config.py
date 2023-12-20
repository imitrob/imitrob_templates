

PickTaskConfig = {
    'id': 80,
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
    'execution_config_params': {
        'move_near_z_offset': 0.1,
        'move_final_z_offset': 0.04,
    }
}

PointTaskConfig = {
    'id': 64,
    'name': 'point',
    'pars_compulsary': ['target_action', 'target_object'],
    'pars_voluntary': [],
    # inverted soft-requirements
    'target_object_penalization': {
        'reachable':   0.4, # When object is not reachable, I still may want point to it, but the constraint action is penalized
        'pickable':    1.0, 
        'stackable':   1.0, 
        'pushable':    1.0, 
        'full-stack':  1.0, # When the object is full it can be still pointed to
        'full-liquid': 1.0,
        'glued':       1.0,
        },
    # hard requirements
    'requirements': {'+': ['reachable'], '-': []},
    'execution_config_params': {
        'move_near_z_offset': 0.25,
    }
}

PassTaskConfig = {
    'id': 81,
    'name': 'point',
    'pars_compulsary': ['target_action', 'target_object'],
    'pars_voluntary': [],
    # inverted soft-requirements
    'target_object_penalization': {
        'reachable':   0.4,
        'pickable':    0.0, 
        'stackable':   1.0, 
        'pushable':    1.0, 
        'full-stack':  1.0, 
        'full-liquid': 1.0,
        'glued':       0.0,
        },
    # hard requirements
    'requirements': {'+': ['reachable', 'pickable'], '-': ['glued']},
    'execution_config_params': {
        'move_near_z_offset': 0.1,
        'move_final_z_offset': 0.04,
        'pass_to_position': [0.5, 0.2, 0.3]
    }
}

ReleaseTaskConfig = {
    'id': 128,
    'name': 'release',
    'pars_compulsary': ['target_action'],
    'pars_voluntary': [],
    # inverted soft-requirements
    'target_object_penalization': {
        'reachable':   1.0,
        'pickable':    1.0, 
        'stackable':   1.0, 
        'pushable':    1.0, 
        'full-stack':  1.0, 
        'full-liquid': 1.0,
        'glued':       1.0,
        },
    # hard requirements
    'requirements': {'+': [], '-': []},
    'execution_config_params': {
        'free_space_z_offset': 0.05,
    }
}
