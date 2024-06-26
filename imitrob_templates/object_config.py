import numpy as np

def to_default_name(name):
    if 'test_CUBE' in name:
        return 'cube_holes'
    
    if '_od_' in name: # it is crow object
        name = name.split("_od_")[0]
        
    name = name.replace("_"," ")
        
    return name


OFFSETS = {
'sugar box': 0.085+0.04, 'cracker box': 0.107+0.03, 'pudding box': 0.045+0.07,
'mustard bottle': 0.095+0.07, 'bowl': 0.05, 'potted meat can': 0.02, 'foam brick': 0.0,
'tomato soup can': 0.03, 'drawer cabinet': 0.3, 'mug': 0.03, 'cube': 0.02,
'wheel': 0.01,
'cube holes': 0.01,
'cube': 0.01,
'test_CUBE': 0.01,
'drawer': 0.02, #0.025
'can': 0.03,
'crackers': 0.04
}

def get_z_offset_from_center(name):
    """Picks predefined offset from the OFFSET dict,
      - crow object names e.g. cube_holes_od_1
        are converted to: cube_holes 

    Args:
        name (String): name of the object
    """    
    
    name = to_default_name(name)

    if name in OFFSETS:
        return OFFSETS[name]
    else:
        print(f"WARNING: get_z_offset_from_center(): Not found object name: {name}")
        return 0.1


OFFSETS_Z_ROT = {
'sugar box': 0.0, 'cracker box': 0.0, 'pudding box': np.pi/2,
'mustard bottle': 0.0, 'bowl': 0.0, 'potted meat can': np.pi/2,
'foam brick': 0.0, 'drawer': 0.0, 'mug': 0.0,
'tomato soup can': 0.0, 'cube': 0.0,
'wheel': 0.0,
'cube holes': 0.0,
'can': np.pi/2,
}

def get_z_offset_rot(name):
    """Picks predefined offset from the OFFSET_Z_ROT dict
     - crow object names e.g. cube_holes_od_1
       are converted to: cube_holes

    Args:
        name (String): name of the object
    """    

    name = to_default_name(name)

    if name in OFFSETS_Z_ROT:
        # OFFSETS_Z_ROT has config data about all object offsets
        return OFFSETS_Z_ROT[name]
    else: # Use simulator
        print(f"WARNING: get_quaternion_eef(): Not found object name: {name}")
        return 0.0
    

def get_static_properties(name):
    """These are constant and defined based on real object

    Args:
        name (Str): Object name

    Returns:
        dict: static_properties
    """    
    
    properties_dict = {
        'cube_holes': { 
            'name': 'box',
            'size': 0.04, # [m]
            'roundness-top': 0.9, # [normalized belief rate]
            'weight': 0.04, # [kg]
            'contains': 0., # normalized rate being full 
            'contain_item': False, # how many items contains
            'types': ['object'],
            'glued': False
        },
        'cube': { 
            'name': 'box',
            'size': 0.04, # [m]
            'roundness-top': 0.9, # [normalized belief rate]
            'weight': 0.04, # [kg]
            'contains': 0., # normalized rate being full 
            'contain_item': False, # how many items contains
            'types': ['object'],
            'glued': False
        },
        'wheel': { 
            'name': 'box',
            'size': 0.05, # [m]
            'roundness-top': 1.0, # [normalized belief rate]
            'weight': 0.03, # [kg]
            'contains': 0., # normalized rate being full 
            'contain_item': False, # how many items contains
            'types': ['object'],
            'glued': False
        }
    }

    name = to_default_name(name)

    if name in properties_dict:
        return properties_dict[name] 
    else:
        print(f"WARNING: get_static_properties(): Not found object name: {name}")
        return properties_dict['cube holes'] 
    
            

NAME2TYPE = {
# "<name>": "<type>",
"master chef can": "object",
"cracker box": "object",
"sugar box": "object",
"tomato soup can": "cup",
"mustard bottle": "cup",
"tuna fish can": "cup",
"pudding box": "object",
"gelatin box": "object",
"potted meat can": "object",
"banana": "object",
"pitcher base": "object",
"bleach cleanser": "object",
"bowl": "drawer",
"mug": "cup",
"power drill": "object",
"wood block": "object",
"scissors": "object",
"large marker": "object",
"medium_clamp": "object",
"large clamp": "object",
"foam brick": "object",
}

COSYPOSE2NAME = {
    "obj_000001": "master chef can",
    "obj_000002": "cracker box",
    "obj_000003": "sugar box",
    "obj_000004": "tomato soup can",
    "obj_000005": "mustard bottle",
    "obj_000006": "tuna fish can",
    "obj_000007": "pudding box",
    "obj_000008": "gelatin box",
    "obj_000009": "potted meat can",
    "obj_000010": "banana",
    "obj_000011": "pitcher base",
    "obj_000012": "bleach cleanser",
    "obj_000013": "bowl",
    "obj_000014": "mug",
    "obj_000015": "power drill",
    "obj_000016": "wood block",
    "obj_000017": "scissors",
    "obj_000018": "large marker",
    "obj_000019": "medium_clamp",
    "obj_000020": "large clamp",
    "obj_000021": "foam brick",
}

COSYPOSE_TRANSFORM = {
    "obj_000001": (np.array([0.0, 0.0, -0.070072]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000002": (np.array([0.0, 0.0, -0.112]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000003": (np.array([0.0, 0.0, -0.088008]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000004": (np.array([0.0, 0.0, -0.051018]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000005": (np.array([0.0, 0.0, -0.103]), np.array([0.0, 0.0, -0.20364175114, 0.97904547248])),
    "obj_000006": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000007": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000008": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000009": (np.array([0.0, 0.0, -0.05]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000010": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000011": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000012": (np.array([0.0, 0.0, -0.13]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000013": (np.array([0.0, 0.0, -0.027485]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000014": (np.array([0.0, 0.0, -0.045]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000015": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000016": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000017": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000018": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000019": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000020": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000021": (np.array([0.0, 0.0, -0.028]), np.array([0.0, 0.0, 0.0, 1.0])),
}

COSYPOSE_TRANSFORM_DEFAULT = {
    "obj_000001": (np.array([0.0, 0.0, -0.070072]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000002": (np.array([0.0, 0.0, -0.106743]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000003": (np.array([0.0, 0.0, -0.0880075]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000004": (np.array([0.0, 0.0, -0.0510185]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000005": (np.array([0.0, 0.0, -0.095704]), np.array([0.0, 0.0, -0.20364175114, 0.97904547248])),
    "obj_000006": (np.array([0.0, 0.0, -0.0167555]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000007": (np.array([0.0, 0.0, -0.019414]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000008": (np.array([0.0, 0.0, -0.01506]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000009": (np.array([0.0, 0.0, -0.0418185]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000010": (np.array([0.0, 0.0, -0.018335]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000011": (np.array([0.0, 0.0, -0.12132]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000012": (np.array([0.0, 0.0, -0.12532]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000013": (np.array([0.0, 0.0, -0.027485]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000014": (np.array([0.0, 0.0, -0.040692]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000015": (np.array([0.0, 0.0, -0.0286585]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000016": (np.array([0.0, 0.0, -0.102945]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000017": (np.array([0.0, 0.0, -0.007858]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000018": (np.array([0.0, 0.0, -0.0094385]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000019": (np.array([0.0, 0.0, -0.019575]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000020": (np.array([0.0, 0.0, -0.0181665]), np.array([0.0, 0.0, 0.0, 1.0])),
    "obj_000021": (np.array([0.0, 0.0, -0.0255655]), np.array([0.0, 0.0, 0.0, 1.0])),
}

COSYPOSE_BBOX = {
    "obj_000001": {
        "x": 0.10239100456237793,
        "y": 0.10231000185012817,
        "z": 0.14016400277614594
    },
    "obj_000002": {
        "x": 0.07181200385093689,
        "y": 0.1640549898147583,
        "z": 0.21345899999141693
    },
    "obj_000003": {
        "x": 0.049459002912044525,
        "y": 0.09414799511432648,
        "z": 0.17601999640464783
    },
    "obj_000004": {
        "x": 0.067859,
        "y": 0.067714,
        "z": 0.101843
    },
    "obj_000005": {
        "x": 0.09598095715045929,
        "y": 0.058198802173137665,
        "z": 0.1913120150566101
    },
    "obj_000006": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000007": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000008": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000009": {
        "x": 0.10211099684238434,
        "y": 0.06012200191617012,
        "z": 0.08350799977779388
    },
    "obj_000010": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000011": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000012": {
        "x": 0.10242000222206116,
        "y": 0.06771500408649445,
        "z": 0.2505960166454315
    },
    "obj_000013": {
        "x": 0.16134102642536163,
        "y": 0.16107700765132904,
        "z": 0.05502000078558922
    },
    "obj_000014": {
        "x": 0.1169310212135315,
        "y": 0.09294901043176651,
        "z": 0.08125299960374832
    },
    "obj_000015": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000016": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000017": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000018": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000019": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000020": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "obj_000021": {
        "x": 0.05254700034856796,
        "y": 0.07788799703121185,
        "z": 0.05120500177145004
    }
}

CAMERA_DATA = {
    'intrinsic': np.array([
        [624.2282702957912, 0.0, 640.0],
        [0.0, 624.2282702957912, 360.0],
        [0.0, 0.0, 1.0]
    ]),
    'extrinsic':{
        'pos': np.array([1.1398999691009521, 0.04690000042319298, 0.2775000035762787]),
        'ori': np.array([0.5590000152587891, 0.4250999987125397, 0.4602999985218048, 0.5430999994277954])
    }
}
