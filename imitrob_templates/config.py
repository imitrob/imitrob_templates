

PickTaskConfig = {
    'id': 1,
    'name': 'pick',
    # 'synonyms': ['pick', 'PICK_TASK', 'PickTask', 'pick up', 'lift', 'use', 'pick-up'],
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
    'target_storage_penalization': {},
    # hard requirements
    'requirements': {'+': ['reachable', 'pickable'], '-': ['glued']},
    'execution_config_params': {
        'move_near_z_offset': 0.1,
        'move_final_z_offset': 0.04,
    },
    
    'mm_pars_compulsary': ['template', 'selections'],
}

PointTaskConfig = {
    'id': 2,
    'name': 'point',
    # 'synonyms': ['point', 'POINT_TASK', 'PointTask'],
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
    'target_storage_penalization': {},
    # hard requirements
    'requirements': {'+': ['reachable'], '-': []},
    'execution_config_params': {
        'move_near_z_offset': 0.25,
    },
    
    'mm_pars_compulsary': ['template', 'selections'],
}

PassTaskConfig = {
    'id': 3,
    'name': 'pass-me',
    # 'synonyms': ['pass', 'pass me', 'give me', 'bring', 'need', 'pass-me'],
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
    'target_storage_penalization': {},
    # hard requirements
    'requirements': {'+': ['reachable', 'pickable'], '-': ['glued']},
    'execution_config_params': {
        'move_near_z_offset': 0.1,
        'move_final_z_offset': 0.04,
        'pass_to_position': [0.5, 0.2, 0.3]
    },
    
    'mm_pars_compulsary': ['template', 'selections'],
}

ReleaseTaskConfig = {
    'id': 4,
    'name': 'release',
    # 'synonyms': ['release', 'place', 'put down'],
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
    'target_storage_penalization': {},
    # hard requirements
    'requirements': {'+': [], '-': []},
    'execution_config_params': {
        'free_space_z_offset': 0.05,
    },
    
    'mm_pars_compulsary': ['template'],
}




PourTaskConfig = {
    
    'pars_compulsary': ['target_action', 'target_object', 'target_storage'],
    'pars_voluntary': [],
    
    # 'synonyms': ['pour', 'pour into'],
    'mm_pars_compulsary': ['template', 'selections', 'storages'],
}

StopTaskConfig = {
    'id': 0,
    'name': 'stop',
    'pars_compulsary': ['target_action'],
    'pars_voluntary': [],
    
    'target_object_penalization': {}, # shouldn't be accessed
    'target_storage_penalization': {},
    
    'requirements': {'+': [], '-': []},
    'execution_config_params': {
    },
    
    'mm_pars_compulsary': ['template'],
}

MoveUpTaskConfig = {
    'id': 11,
    'name': 'move-up',
    'pars_compulsary': ['target_action'],
    'pars_voluntary': [],
    
    'target_object_penalization': {}, # shouldn't be accessed
    'target_storage_penalization': {},
    
    'requirements': {'+': [], '-': []},
    'execution_config_params': {
    },
    
    'mm_pars_compulsary': ['template'],
}