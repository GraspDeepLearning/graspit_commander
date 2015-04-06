"""graspit_commander.py contains the main functions of the graspit_commander package for
   interacting with Graspit! through Python via ros service calls."""


from graspit_ros_planning_msgs.srv import (
    GenerateGraspRemoteDB,
    GenerateGraspsLocalModel,
    GenerateGraspsLocalModelRequest,
    SimulateScan,
    SimulateScanRequest)

from std_msgs.msg import String
from geometry_msgs.msg import Pose
import rospy


#18633 - 18808
#drill = 18741
def generate_grasp_remote_db(model_id=18741,
                             energy_low_threshold=0,
                             energy_high_threshold=1,
                             reject_fingertip_collision=False,
                             request_tabletop=False,
                             tabletop_file_name="table.xml"):
    """Generates a single grasp in Graspit using a model from the object database.

    :param int model_id:
        the id of the model to be loaded
    :param float energy_low_threshold:
        low threshold for the graps energy, energy lower than this value won't be returned
    :param float energy_high_threshold:
        high threshold for the grasp energy, energy higher than this value will be rejected
    :param bool reject_fingertip_collision:
        if true, then fingertip collision is not allowed for the bad grasp, if false, only fingertip
        collision grasp will be returned
    :param bool request_tabletop:
        if true, then any grasp that collides with the tabltop won't be accepted
    :param str tabletop_file_name:
        the tabletop file name

    :returns: grasp response
    :rtype: GenerateGraspRemoteDB message
    """

    generate_grasp_proxy = rospy.ServiceProxy('/ros_graspit_interface/generate_grasp_remote_db',
                                              GenerateGraspRemoteDB)

    resp = generate_grasp_proxy(model_id,
                                energy_low_threshold,
                                energy_high_threshold,
                                reject_fingertip_collision,
                                request_tabletop,
                                tabletop_file_name)
    return resp


def generate_grasps_local_model(model_name="book.xml",
                                energy_low_threshold=0,
                                energy_high_threshold=1,
                                reject_fingertip_collision=False,
                                request_tabletop=False,
                                tabletop_file_name="table.xml"):
    """Generates a list of grasps in Graspit from a local model.

    :param str model_name:
        the file name of the model
    :param float energy_low_threshold:
        low threshold for the graps energy, energy lower than this value won't be returned
    :param float energy_high_threshold:
        high threshold for the grasp energy, energy higher than this value will be rejected
    :param bool reject_fingertip_collision:
        if true, then fingertip collision is not allowed for the bad grasp, if false, only fingertip
        collision grasp will be returned
    :param bool request_tabletop:
        if true, then any grasp that collides with the tabltop won't be accepted
    :param str tabletop_file_name:
        the tabletop file name

    :returns: [grasp response]
    :rtype: array of grasp messages
    """

    generate_grasps_proxy = rospy.ServiceProxy('/ros_graspit_interface/generate_grasps_local_model',
                                               GenerateGraspsLocalModel)

    request = GenerateGraspsLocalModelRequest()
    request.model_name = String(model_name)
    request.energy_low_threshold = energy_low_threshold
    request.energy_high_threshold = energy_high_threshold
    request.reject_fingertip_collision = bool(reject_fingertip_collision)
    request.request_tabletop = bool(request_tabletop)
    request.tabletop_file_name = str(tabletop_file_name)

    resp = generate_grasps_proxy(request)
    return resp


def simulate_scan(pose=Pose(), request_ray_directions=False):
    """Simulates a scan...

    TODO: Write better docstring.
    """
    generate_scan_proxy = rospy.ServiceProxy('/ros_graspit_interface/simulate_scan', SimulateScan)

    request = SimulateScanRequest()
    request.scanner_pose = pose
    request.request_ray_directions = bool(request_ray_directions)

    resp = generate_scan_proxy(request)
    return resp
