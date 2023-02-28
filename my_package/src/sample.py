#!/usr/bin/env python
# Lib
import rospy

from niryo_robot_utils import NiryoRosWrapperException, NiryoActionClient, NiryoTopicValue, AbstractNiryoRosWrapper

# Command Status
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from conveyor_interface.msg import ConveyorFeedbackArray
from niryo_robot_msgs.msg import HardwareStatus, RobotState, RPY
from niryo_robot_rpi.msg import DigitalIO, DigitalIOState, AnalogIO, AnalogIOState
from niryo_robot_status.msg import RobotStatus

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList, SetBool, SetInt, Trigger, Ping, SetFloat

# Actions
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from niryo_robot_arm_commander.msg import ArmMoveCommand, RobotMoveGoal, RobotMoveAction

# Enums
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import *

from niryo_robot_tools_commander.api import ToolsRosWrapper, ToolID
class NiryoRosWrapper(AbstractNiryoRosWrapper):

    def __init__(self):
        super(NiryoRosWrapper, self).__init__()
        # - Getting ROS parameters
        self.__service_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/service_timeout")
        self.__simulation_mode = rospy.get_param("/niryo_robot/simulation_mode")
        self.__hardware_version = rospy.get_param("/niryo_robot/hardware_version")

        if self.__hardware_version in ['ned', 'ned2']:
            self.__node_name = rospy.get_name()
            self.__ping_ros_wrapper_srv = rospy.Service("~/ping", Trigger, self.__ping_ros_wrapper_callback)
            rospy.wait_for_service("/niryo_robot_status/ping_ros_wrapper", timeout=5)
            self.__advertise_ros_wrapper_srv = rospy.ServiceProxy("/niryo_robot_status/ping_ros_wrapper", Ping)
            self.__advertise_ros_wrapper_srv(self.__node_name, True)
            rospy.on_shutdown(self.__advertise_stop)

        # - Publishers
        # Highlight publisher (to highlight blocks in Blockly interface)
        self.__highlight_block_publisher = rospy.Publisher('/niryo_robot_blockly/highlight_block', String,
                                                           queue_size=10)

        # Break point publisher (for break point blocks in Blockly interface)
        self.__break_point_publisher = rospy.Publisher('/niryo_robot_blockly/break_point', Int32, queue_size=10)

        # -- Subscribers
        # - Pose
        self.__joints_ntv = NiryoTopicValue('/joint_states', JointState)
        self.__pose_ntv = NiryoTopicValue('/niryo_robot/robot_state', RobotState)

        # - Hardware
        self.__learning_mode_on_ntv = NiryoTopicValue('/niryo_robot/learning_mode/state', Bool)
        self.__hw_status_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/hardware_status', HardwareStatus)
        self.__digital_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/digital_io_state', DigitalIOState)
        self.__analog_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/analog_io_state', AnalogIOState)
        self.__max_velocity_scaling_factor_ntv = NiryoTopicValue('/niryo_robot/max_velocity_scaling_factor', Int32)

        # - Vision
        self.__compressed_image_message_ntv = NiryoTopicValue('/niryo_robot_vision/compressed_video_stream',
                                                              CompressedImage, queue_size=1)
        self.__camera_intrinsics_message_ntv = NiryoTopicValue('/niryo_robot_vision/camera_intrinsics',
                                                               CameraInfo, queue_size=1)
        # - Conveyor
        self.__conveyors_feedback_ntv = NiryoTopicValue('/niryo_robot/conveyor/feedback', ConveyorFeedbackArray)

        # Software
        self.__software_version_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/software_version',
                                                      SoftwareVersion,
                                                      queue_size=1)

        # - Action server
        # Robot action
        self.__robot_action_nac = NiryoActionClient('/niryo_robot_arm_commander/robot_action',
                                                    RobotMoveAction, RobotMoveGoal)

        self.__follow_joint_traj_nac = NiryoActionClient(
            rospy.get_param("/niryo_robot_arm_commander/joint_controller_name") + "/follow_joint_trajectory",
            FollowJointTrajectoryAction, FollowJointTrajectoryGoal)

        self.__tools = ToolsRosWrapper(self.__service_timeout)

        # database
        from niryo_robot_database.api import DatabaseRosWrapper
        self.__database = DatabaseRosWrapper(self.__service_timeout)

        # system_api_client
        from niryo_robot_system_api_client.api import SystemApiClientRosWrapper
        self.__system_api_client = SystemApiClientRosWrapper(self.__service_timeout)

        from niryo_robot_status.api import RobotStatusRosWrapper
        self.__robot_status = RobotStatusRosWrapper(self.__service_timeout)

        if self.__hardware_version == 'ned2':
            from niryo_robot_python_ros_wrapper.custom_button_ros_wrapper import CustomButtonRosWrapper
            from niryo_robot_led_ring.api import LedRingRosWrapper
            from niryo_robot_sound.api import SoundRosWrapper

            # Led Ring
            self.__led_ring = LedRingRosWrapper(self.__hardware_version, self.__service_timeout)
            # Sound
            self.__sound = SoundRosWrapper(self.__hardware_version, self.__service_timeout)
            # - Custom button
            self.__custom_button = CustomButtonRosWrapper(self.__hardware_version)
        else:
            self.__led_ring = self.__sound = self.__custom_button = None

        rospy.loginfo("Python ROS Wrapper ready")

    def __advertise_stop(self):
        if self.__hardware_version in ['ned', 'ned2']:
            try:
                self.__advertise_ros_wrapper_srv(self.__node_name, False)
            except [rospy.ServiceException, rospy.ROSException]:
                pass

    def __ping_ros_wrapper_callback(self):
        return CommandStatus.SUCCESS, self.__node_name

    @classmethod
    def wait_for_nodes_initialization(cls, simulation_mode=False):
        params_checked = [
            '/niryo_robot_poses_handlers/initialized',
            '/niryo_robot_arm_commander/initialized',
        ]
        while not all([rospy.has_param(param) for param in params_checked]):
            rospy.sleep(0.1)
        if simulation_mode:
            rospy.sleep(1)  # Waiting to be sure Gazebo is open

    # -- Publishers
    # Blockly
    def highlight_block(self, block_id):
        msg = String()
        msg.data = block_id
        self.__highlight_block_publisher.publish(msg)

    def break_point(self):
        import sys

        msg = Int32()
        msg.data = 1
        self.__break_point_publisher.publish(msg)

        # Close program
        sys.exit()

    # -- Main Purpose
    def request_new_calibration(self):
        """
        Calls service to set the request calibration value. If failed, raises NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        try:
            return self._call_service('/niryo_robot/joints_interface/request_new_calibration', Trigger)
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(str(e))
    def move_joints(self, j1, j2, j3, j4, j5, j6):
        """
        Executes Move joints action

        :param j1:
        :type j1: float
        :param j2:
        :type j2: float
        :param j3:
        :type j3: float
        :param j4:
        :type j4: float
        :param j5:
        :type j5: float
        :param j6:
        :type j6: float
        :return: status, message
        :rtype: (int, str)
        """
        cmd = ArmMoveCommand(cmd_type=ArmMoveCommand.JOINTS, joints=[j1, j2, j3, j4, j5, j6])
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)
if __name__ == '__main__':
    go = NiryoRosWrapper()
    go.move_joints(0.1, -0.2, 0.0, 1.1, -0.5, 0.2)

