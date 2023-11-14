#include <cstdio>
#include "coveragecontrol_sim/sim.hpp"

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	auto node = std::make_shared<CoverageControlSim>();
	executor.add_node(node);
	executor.spin();

  rclcpp::shutdown();
  return 0;
}
