#include <dragoman_fsw_sim/mobile_servicing_system_comm_udp.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MobileServicingSystemCommUdp>();
  
  int tlm_ms = 2000;
  int cmd_ms = 100;
  
  node->initRobotComm();
  
  if(!node->initUdpComm())
  {
    RCLCPP_ERROR(node->get_logger(), "Error initializing communication");
    return 1;
  }
  
  node->initRest(tlm_ms, cmd_ms);  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
