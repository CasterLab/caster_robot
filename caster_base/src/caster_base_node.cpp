#include <caster_base/caster_hardware.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_base_node");

  ros::AsyncSpinner spinner(3);

  ros::NodeHandle nh, private_nh("~");
  std::string node_name = ros::this_node::getName();

  iqr::CasterHardware caster;
  caster.Initialize(node_name, nh, private_nh);

  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}
