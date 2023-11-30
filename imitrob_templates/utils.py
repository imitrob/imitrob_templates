
''' Note: Might be moved to some better place '''
from spatialmath import UnitQuaternion
import spatialmath as sm

def get_quaternion_eef(q_, name):
    ''' Based on CosyPose, where each object (name) has OFFSET
    '''
    try:
        # ycb_data.OFFSETS_Z_ROT has config data about all object offsets
        offset_z_rot = ycb_data.OFFSETS_Z_ROT[name.replace("_"," ")]
    except KeyError: # Use simulator
        print(f"get_quaternion_eef - Not found object name: {name}")
        offset_z_rot = 0.

    q = UnitQuaternion([0.0,0.0,1.0,0.0]) # w,x,y,z
    q_2 = UnitQuaternion([q_[3], *q_[0:3]])

    rot = sm.SO3(q.R) * sm.SO3.Rz(q_2.rpy()[2]-np.pi/2+offset_z_rot)
    return UnitQuaternion(rot).vec_xyzs