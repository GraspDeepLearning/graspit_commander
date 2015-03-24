

from graspit_ros_planning_msgs.srv import *
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import rospy


#18633 - 18808
#drill = 18741
def generate_grasp(model_id=18741,
                   energy_low_threshold=0,
                   energy_high_threshold=1,
                   reject_fingertip_collision=False,
                   request_tabletop=False,
                   tabletop_file_name="table.xml"):

    generate_grasp_proxy = rospy.ServiceProxy('/ros_graspit_interface/generate_grasp', GenerateGrasp)

    resp = generate_grasp_proxy(model_id,
                                energy_low_threshold,
                                energy_high_threshold,
                                reject_fingertip_collision,
                                request_tabletop,
                                tabletop_file_name)
    return resp


def generate_grasps(model_name="book.xml",
                   energy_low_threshold=0,
                   energy_high_threshold=1,
                   reject_fingertip_collision=False,
                   request_tabletop=False,
                   tabletop_file_name="table.xml"):

    generate_grasps_proxy = rospy.ServiceProxy('/ros_graspit_interface/generate_grasps', GenerateGrasps)

    request = GenerateGraspsRequest()
    request.model_name = String(model_name)
    request.energy_low_threshold = energy_low_threshold
    request.energy_high_threshold = energy_high_threshold
    request.reject_fingertip_collision = bool(reject_fingertip_collision)
    request.request_tabletop = bool(request_tabletop)
    request.tabletop_file_name = str(tabletop_file_name)

    resp = generate_grasps_proxy(request)
    return resp


def simulate_scan(pose=Pose(), request_ray_directions=False):

    generate_scan_proxy = rospy.ServiceProxy('/ros_graspit_interface/simulate_scan', SimulateScan)

    request = SimulateScanRequest()
    request.scanner_pose = pose
    request.request_ray_directions = bool(request_ray_directions)

    resp = generate_scan_proxy(request)
    return resp

if __name__ == "__main__":

    rospy.init_node("graspit_commander")
    import IPython
    IPython.embed()