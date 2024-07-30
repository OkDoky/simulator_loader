
from typing import Union
import rospy
import math
import random
import tf
from geometry_msgs.msg import PoseStamped, Pose2D
from flatland_msgs.msg import MoveModelMsg
from std_srvs.srv import Empty
from utils.pose_transform import Pose2D2PoseStamped, PoseStamped2Pose2D
from cfg.config import Config

class RobotManager:
    def __init__(self, ns, model_name, map_nanger):
        # set default values
        self.ns = ns
        self.model_name = ns + "/" + model_name
        footprint = rospy.get_param("dmvm/FootprintProcessor/footprint", [])
        if len(eval(footprint)):
            max_height = max([abs(point[1]) for point in footprint])
            max_width = max([abs(point[0]) for point in footprint])
            self.robot_radius = math.hypot(max_height, max_width)
        else:
            self.robot_radius = 0.105  ## default burger robot
        self.map_frame = rospy.get_param("global_planner/costmap/global_frame", "map")
        self.map_manager = map_manager
        
        # init publisher
        self.pubs = {}
        self.pubs["move_robot"] = rospy.Publisher("move_model", MoveModelMsg, queue_size=1)
        self.pubs["goal"] = rospy.Publisher("goal", PoseStamped, queue_size=1)

        ## only for test
        test_mode = rospy.get_param("/test_mode", False)
        if test_mode:
            rospy.Service("reset_robot", Empty, self.resetCallback)



    def generateSpawnAndGoalPose(self, forbidden_zone):
        safety_radius = self.robot_radius + Config.RobotConfig.RobotSafeDist
        spawn_pose = Pose2D(*self.getRandomPose(safety_radius, forbidden_zone))
        goal_pose = Pose2D(*self.getRandomPose(safety_radius, [*forbidden_zone, spawn_pose]))
        return spawn_pose, goal_pose

    def getRandomPose(self, safety_radius, forbidden_zone):
        return self.map_manager.get_random_pos_on_map(safety_radius, forbidden_zone)

    def getStaticPose(self, pos: Union[Pose2D, PoseStamped], output_type: str="PoseStamped"):
        assert isinstance(pos, Union[Pose2D, PoseStamped])
        
        if output_type == "PoseStamped":
            if isinstance(pos, Pose2D):
                static_pose = 
                    Pose2D2PoseStamped(pose=pos, frame_id=self.map_frame)
            else:
                static_pose = pos
                static_pose.header.stamp = rospy.get_rostime()
                static_pose.header.frame_id = self.map_frame
        
        elif output_type == "Pose2D":
            if isinstance(pos, Pose2D):
                static_pose = pos
            else:
                static_pose = Pose2D2PoseStamped(pose=pos)

        return static_pose
    
    def reset(self, forbidden_zone=[]):
        spawn_pos, goal_pos = self.generateSpawnAndGoalPose(forbidden_zone)
        self.moveRobot(spawn_pos)
        self.setGoal(goal_pos)

    def moveRobot(self, pos: Union[Pose2D, PoseStamped]):
        assert isinstance(pos, Union[Pose2D, PoseStamped])

        move_pose_msg = MoveModelMsg()
        move_pose_msg.name = self.model_name

        if isinstance(pos, Pose2D):
            move_pose_msg.pose = pos
            self.pubs["move_robot"].publish(move_pose_msg)
        else:
            move_pose_msg.pose = PoseStamped2Pose2D(pos)
            self.pubs["move_robot"].publish(move_pose_msg)

    def setGoal(self, goal: Union[Pose2D, PoseStamped]):
        assert isinstance(goal, Union[Pose2D, PoseStamped])
        if isinstance(goal, Pose2D):
            self.pubs["goal"].publish(Pose2D2PoseStamped(goal, self.map_frame))
        else:
            self.pubs["goal"].publish(goal)

    def resetCallback(self, req):
        self.reset()
        return