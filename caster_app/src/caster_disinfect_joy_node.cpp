#include <ros/ros.h>
#include <string>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <aruco_msgs/MarkerArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <caster_man_app/PickGift.h>


class CasterDisinfect
{
  public:
    ros::Publisher init_pub;
    CasterDisinfect();
};

bool move_goal_flag, init_pose = false, stop = false;
ros::Publisher init_pub;
geometry_msgs::Pose pose4112, pose4111, pose_west_hall, pose4101, pose_liabray1, pose4086, pose_liabray2, pose_liabray3, pose4102, pose4062, pose_worldcup,
  pose4085, pose4031, pose_community1, pose_community2, pose_community_youth_meeting, pose4021, pose_east_hall, pose4034, pose4051, pose1, pose2, pose3, pose4, pose5;
void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  // ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  // ROS_INFO_STREAM("Move base done callback");
}

void MoveToGoal(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &move_base_client, geometry_msgs::Pose pose) {
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "map";
  mb_goal.target_pose.pose = pose;

  move_goal_flag = false;
  move_base_client.sendGoal(mb_goal,
            boost::bind(&MovebaseDoneCallback, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&MovebaseFeedbackCallback, _1));

  while(move_goal_flag==false) {
    ros::WallDuration(1.0).sleep();
    ROS_INFO_STREAM("moving...");
  }

  ROS_INFO_STREAM("get goal");
}

void GetGoalPose(std::string goal_name, ros::NodeHandle &private_nh, geometry_msgs::Pose &pose) {
  std::vector<float> pose_vector(3), orientation_vector(4);

  private_nh.getParam("target_goal/"+goal_name+"/pose", pose_vector);
  private_nh.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

  pose.position.x = pose_vector[0];
  pose.position.y = pose_vector[1];
  pose.position.z = pose_vector[2];
  pose.orientation.x = orientation_vector[0];
  pose.orientation.y = orientation_vector[1];
  pose.orientation.z = orientation_vector[2];
  pose.orientation.w = orientation_vector[3];
}

bool InitialPose(caster_man_app::PickGift::Request &req, caster_man_app::PickGift::Response &res) {
  if(!init_pose) {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose.position.x = 24.1389350891;
    initial_pose.pose.pose.position.y = -65.5240631104;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.709878601966;
    initial_pose.pose.pose.orientation.w = 0.704324052174;
    init_pub.publish(initial_pose);
    init_pose = true;
    return true;
  }
}

bool Stop(caster_man_app::PickGift::Request &req, caster_man_app::PickGift::Response &res) {
  stop = true;
  return true;
}

bool Disinfect(caster_man_app::PickGift::Request &req, caster_man_app::PickGift::Response &res) {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);
  // Wait a bit for ROS things to initialize
  while(!stop) {
    ROS_INFO("Moveto east hall goal");
    MoveToGoal(move_base_client, pose_east_hall);
    ROS_INFO("Moveto community youth meeting goal");
    MoveToGoal(move_base_client, pose_community_youth_meeting);
    ROS_INFO("Moveto 402-1 goal");
    MoveToGoal(move_base_client, pose4021);
    ROS_INFO("Moveto 403-4 goal");
    MoveToGoal(move_base_client, pose4034);
    ROS_INFO("Moveto 405-1 goal");
    MoveToGoal(move_base_client, pose4051);
    ROS_INFO("Moveto community 1 goal");
    MoveToGoal(move_base_client, pose_community1);
    // ROS_INFO("Moveto community 2 goal");
    // MoveToGoal(move_base_client, pose_community2);
    ROS_INFO("Moveto 403-1 goal");
    MoveToGoal(move_base_client, pose4031);
    ROS_INFO("Moveto 408-5 goal");
    MoveToGoal(move_base_client, pose4085);
    ROS_INFO("Moveto 406-2 goal");
    MoveToGoal(move_base_client, pose4062);
    ROS_INFO("Moveto 410-2 goal");
    MoveToGoal(move_base_client, pose4102);
    // ROS_INFO("Moveto liabray 1 goal");
    // MoveToGoal(move_base_client, pose_liabray1);
    ROS_INFO("Moveto 408-6 goal");
    MoveToGoal(move_base_client, pose4086);
    ROS_INFO("Moveto liabray 2 goal");
    MoveToGoal(move_base_client, pose_liabray2);
    // ROS_INFO("Moveto liabray 3 goal");
    // MoveToGoal(move_base_client, pose_liabray3);
    ROS_INFO("Moveto 411-2 goal");
    MoveToGoal(move_base_client, pose4112);
    ROS_INFO("Moveto 411-1 goal");
    MoveToGoal(move_base_client, pose4111);
    ROS_INFO("Moveto west hall goal");
    MoveToGoal(move_base_client, pose_west_hall);
    ROS_INFO("Moveto 410-1 goal");
    MoveToGoal(move_base_client, pose4101);
    ROS_INFO("Moveto east hall goal");
    MoveToGoal(move_base_client, pose_east_hall);
    // ROS_INFO("Moveto world cup goal");
    // MoveToGoal(move_base_client, pose_worldcup);
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_disinfect_node");
  ros::NodeHandle nh, private_nh("~");

  GetGoalPose("eleven_two", private_nh, pose4112);
  GetGoalPose("eleven_one", private_nh, pose4111);
  GetGoalPose("west_hall", private_nh, pose_west_hall);
  GetGoalPose("liabray_1", private_nh, pose_liabray1);
  GetGoalPose("eight_six", private_nh, pose4086);
  GetGoalPose("liabray_2", private_nh, pose_liabray2);
  GetGoalPose("liabray_3", private_nh, pose_liabray3);
  GetGoalPose("ten_two", private_nh, pose4102);
  GetGoalPose("six_two", private_nh, pose4062);
  GetGoalPose("world_cup", private_nh, pose_worldcup);
  GetGoalPose("eight_five", private_nh, pose4085);
  GetGoalPose("three_one", private_nh, pose4031);
  GetGoalPose("community", private_nh, pose_community1);
  GetGoalPose("community_2", private_nh, pose_community2);
  GetGoalPose("community_youth_meeting", private_nh, pose_community_youth_meeting);
  GetGoalPose("two_one", private_nh, pose4021);
  GetGoalPose("east_hall", private_nh, pose_east_hall);
  GetGoalPose("three_four", private_nh, pose4034);
  GetGoalPose("five_one", private_nh, pose4051);
  GetGoalPose("ten_one", private_nh, pose4101);
  GetGoalPose("one", private_nh, pose1);

  // set up spinner
  ros::AsyncSpinner spinner(4);
  spinner.start();
  init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
  ros::ServiceServer service_disinfect = nh.advertiseService("disinfect",Disinfect);
  ros::ServiceServer service_init = nh.advertiseService("initialpose", InitialPose);
  ros::ServiceServer service_stop = nh.advertiseService("stop", Stop);

  // ros::WallDuration(1.5).sleep();
  ros::waitForShutdown();
  return 0;
}
