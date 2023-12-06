
''' Note: Might be moved to some better place '''
from spatialmath import UnitQuaternion
import spatialmath as sm

import numpy as np
from imitrob_templates.object_config import get_z_offset_rot
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms


def get_quaternion_eef(q_, name):
    ''' Based on CosyPose, where each object (name) has OFFSET
    '''
    offset_z_rot = get_z_offset_rot(name)

    q = UnitQuaternion([0.0,0.0,1.0,0.0]) # w,x,y,z
    q_2 = UnitQuaternion([q_[3], *q_[0:3]])

    rot = sm.SO3(q.R) * sm.SO3.Rz(q_2.rpy()[2]-np.pi/2+offset_z_rot)
    return UnitQuaternion(rot).vec_xyzs