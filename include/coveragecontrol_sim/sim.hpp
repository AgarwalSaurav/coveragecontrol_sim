#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/coverage_system.h>

using namespace std::chrono_literals;
using namespace CoverageControl;

class CoverageControlSim : public rclcpp::Node {

	private:

		std::shared_ptr<CoverageSystem> coverage_system_ptr_;
		std::string package_prefix_;
		Parameters parameters_;
		int buffer_size_ = 10;
		rclcpp::CallbackGroup::SharedPtr cbg_robot_positions_pub_;
		std::vector<rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> robot_pos_pubs_;
		// timers
		std::vector<rclcpp::TimerBase::SharedPtr> robot_pos_pub_timers_;

		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_local_map_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_local_map_pub_timers_;

		// Obstacle maps publisher
		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_obstacle_map_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_obstacle_map_pub_timers_;

		// Publishers for relative neighbor positions for each robot
		std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> robot_neighbor_pos_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbor_pos_pub_timers_;

		// Publishers for relative neighbor ids for each robot
		std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> robot_neighbor_id_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_neighbor_id_pub_timers_;

		// Publish sensor view for each robot
		std::vector<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> robot_sensor_view_pubs_;
		std::vector<rclcpp::TimerBase::SharedPtr> robot_sensor_view_pub_timers_;

		// Publisher for global system map
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr global_map_pub_;
		rclcpp::TimerBase::SharedPtr global_map_pub_timer_;

		// Subscribe to robot positions for each robot
		std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> cmd_pos_subs_;
		rclcpp::CallbackGroup::SharedPtr cbg_cmd_pos_sub_;

	public:

		CoverageControlSim() : Node("coveragecontrol_sim"), coverage_system_ptr_(nullptr) {

			package_prefix_ = ament_index_cpp::get_package_prefix("coveragecontrol_sim");
			std::string env_file = this->declare_parameter<std::string>("env_file", package_prefix_ + "/config/env_params.yaml");
			parameters_ = Parameters(env_file);
			std::string pos_file = this->declare_parameter<std::string>("pos_file", package_prefix_ + "/config/sample.pos");
			std::string idf_file = this->declare_parameter<std::string>("idf_file", package_prefix_ + "/config/sample.idf");

			float time_step = this->declare_parameter<float>("time_step", 0.1);
			std::chrono::milliseconds timeout = std::chrono::milliseconds(int(time_step * 1000));
			buffer_size_ = this->declare_parameter<int>("buffer_size", 10);

			if (pos_file != "" && idf_file != "") {
				WorldIDF world_idf(parameters_, idf_file);
				coverage_system_ptr_ = std::make_shared<CoverageSystem>(parameters_, world_idf, pos_file);
			} else {
				coverage_system_ptr_ = std::make_shared<CoverageSystem>(parameters_, parameters_.pNumFeatures, parameters_.pNumRobots);
			}

			/* auto cbg_robot_pos_pub_opt = rclcpp::PublisherOptions(); */
			/* cbg_robot_pos_pub_opt.callback_group = cbg_robot_positions_pub_; */

			// Make callback group for cmd_pos_subs_ that are mutually exclusive

			cbg_cmd_pos_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto cbg_cmd_pos_opts = rclcpp::SubscriptionOptions();
			cbg_cmd_pos_opts.callback_group = cbg_cmd_pos_sub_;

			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/cmd_pos";
				/* auto sub = this->create_subscription<geometry_msgs::msg::Pose>(topic_name, rclcpp::QoS(buffer_size_), std::bind( &CoverageControlSim::CmdPosCallback, this, std::placeholders::_1), cbg_cmd_pos_opts); */
				// Create sub using lambda function
				auto sub = this->create_subscription<geometry_msgs::msg::Pose>(topic_name, rclcpp::QoS(buffer_size_), [this, robot_id](const geometry_msgs::msg::Pose::ConstSharedPtr msg) -> void {
						// Update robot position
						Point2 robot_pos;
						robot_pos[0] = msg->position.x; robot_pos[1] = msg->position.y;
						coverage_system_ptr_->SetLocalRobotPosition(robot_id, robot_pos);
						}, cbg_cmd_pos_opts);
				cmd_pos_subs_.push_back(sub);
			}

			CreateRobotPosPublishers();
			CreateRobotLocalMapPublishers();
			CreateObstacleMapsPublisher();
			CreateSystemMapPublisher();
			CreateNeigborPosPublisher();
			CreateNeigborIDPublisher();
			CreateSensorViewPublisher();

		}


		void CreateRobotPosPublishers() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				/* RCLCPP_INFO(this->get_logger(), "Creating robot position publisher for robot %d", robot_id); */
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/pose";
				robot_pos_pubs_.push_back(this->create_publisher<geometry_msgs::msg::Pose>(topic_name, buffer_size_));
				auto robot_pos_pub = robot_pos_pubs_.back();
				// Create timer to publish robot position
				auto robot_pos_pub_timer_callback = [this, robot_id, robot_pos_pub]() -> void {
					auto robot_pos = coverage_system_ptr_->GetRobotPosition(robot_id);
					geometry_msgs::msg::Pose robot_pos_msg;
					robot_pos_msg.position.x = robot_pos[0];
					robot_pos_msg.position.y = robot_pos[1];
					robot_pos_pub->publish(robot_pos_msg);
				};
				robot_pos_pub_timers_.push_back(this->create_wall_timer(30ms, robot_pos_pub_timer_callback));
			}
		}

		void CreateRobotLocalMapPublishers() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/local_map";
				robot_local_map_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_local_map_pub = robot_local_map_pubs_.back();
				// Create timer to publish local map
				auto robot_local_map_pub_timer_callback = [this, robot_id, robot_local_map_pub]() -> void {
					auto local_map = coverage_system_ptr_->GetRobotLocalMap(robot_id);
					auto local_map_msg = EigenMatrixRowMajorToFloat32MultiArray(local_map);
					robot_local_map_pub->publish(local_map_msg);
				};
				robot_local_map_pub_timers_.push_back(this->create_wall_timer(30ms, robot_local_map_pub_timer_callback));
			}

		}

		void CreateObstacleMapsPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/obstacle_map";
				robot_obstacle_map_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_obstacle_map_pub = robot_obstacle_map_pubs_.back();
				// Create timer to publish local map
				auto robot_obstacle_map_pub_timer_callback = [this, robot_id, robot_obstacle_map_pub]() -> void {
					auto obstacle_map = coverage_system_ptr_->GetRobotObstacleMap(robot_id);
					auto obstacle_map_msg = EigenMatrixRowMajorToFloat32MultiArray(obstacle_map);
					robot_obstacle_map_pub->publish(obstacle_map_msg);
				};
				robot_obstacle_map_pub_timers_.push_back(this->create_wall_timer(30ms, robot_obstacle_map_pub_timer_callback));
			}
		}

		void CreateNeigborPosPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/neighbors_pos";
				robot_neighbor_pos_pubs_.push_back(this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, buffer_size_));
				auto robot_neighbor_pos_pub = robot_neighbor_pos_pubs_.back();
				auto robot_neighbor_pos_pub_timer_callback = [this, robot_id, robot_neighbor_pos_pub]() -> void {
					auto neighbors_pos = coverage_system_ptr_->GetRelativePositonsNeighbors(robot_id);
					geometry_msgs::msg::PoseArray neighbor_pos_msg;
					for (size_t i = 0; i < neighbors_pos.size(); i++) {
						geometry_msgs::msg::Pose neighbor_pos_i_msg;
						neighbor_pos_i_msg.position.x = neighbors_pos[i][0];
						neighbor_pos_i_msg.position.y = neighbors_pos[i][1];
						neighbor_pos_msg.poses.push_back(neighbor_pos_i_msg);
					}
					robot_neighbor_pos_pub->publish(neighbor_pos_msg);
				};
				robot_neighbor_pos_pub_timers_.push_back(this->create_wall_timer(30ms, robot_neighbor_pos_pub_timer_callback));
			}
		}

		void CreateNeigborIDPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; ++robot_id) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/neighbors_id";
				robot_neighbor_id_pubs_.push_back(this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name, buffer_size_));
				auto robot_neighbor_id_pub = robot_neighbor_id_pubs_.back();
				auto robot_neighbor_id_pub_timer_callback = [this, robot_id, robot_neighbor_id_pub]() -> void {
					auto neighbors_id = coverage_system_ptr_->GetNeighborIDs(robot_id);
					std_msgs::msg::Int32MultiArray neighbor_id_msg;
					neighbor_id_msg.data = neighbors_id;
					/* for (size_t i = 0; i < neighbors_id.size(); i++) { */
					/* 	neighbor_id_msg.data.push_back(neighbors_id[i]); */
					/* } */
					robot_neighbor_id_pub->publish(neighbor_id_msg);
				};
				robot_neighbor_id_pub_timers_.push_back(this->create_wall_timer(30ms, robot_neighbor_id_pub_timer_callback));
			}
		}

		void CreateSensorViewPublisher() {
			for (int robot_id = 0; robot_id < parameters_.pNumRobots; robot_id++) {
				std::string topic_name = "robot_" + std::to_string(robot_id) + "/sensor_view";
				robot_sensor_view_pubs_.push_back(this->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, buffer_size_));
				auto robot_sensor_view_pub = robot_sensor_view_pubs_.back();
				// Create timer to publish local map
				auto robot_sensor_view_pub_timer_callback = [this, robot_id, robot_sensor_view_pub]() -> void {
					MapType sensor_view = coverage_system_ptr_->GetRobotSensorView(robot_id);
					/* if(robot_id == 0) */
					/* 	std::cout << "sensor_view: " << sensor_view.sum() << std::endl; */
					//  Print thread id
					auto sensor_view_msg = EigenMatrixRowMajorToFloat32MultiArray(sensor_view);
					robot_sensor_view_pub->publish(sensor_view_msg);
				};
				robot_sensor_view_pub_timers_.push_back(this->create_wall_timer(30ms, robot_sensor_view_pub_timer_callback));
			}
		}

		void CreateSystemMapPublisher() {
			global_map_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("global_map", buffer_size_);
			auto global_map_pub = global_map_pub_;
			// Create timer to publish local map
			auto global_map_pub_timer_callback = [this, global_map_pub]() -> void {
				auto global_map = coverage_system_ptr_->GetSystemMap();
				auto global_map_msg = EigenMatrixRowMajorToFloat32MultiArray(global_map);
				global_map_pub->publish(global_map_msg);
			};
			global_map_pub_timer_ = this->create_wall_timer(30ms, global_map_pub_timer_callback);
		}


		// Function for eigen matrix to std_msgs::msg::Float32MultiArray conversion
		std_msgs::msg::Float32MultiArray EigenMatrixToFloat32MultiArray(Eigen::MatrixXf matrix) {
			std_msgs::msg::Float32MultiArray msg;
			// Convert eigen matrix to std_msgs::msg::Float32MultiArray
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[0].size = matrix.rows();
			msg.layout.dim[0].stride = matrix.rows();
			msg.layout.dim[0].label = "rows";
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[1].size = matrix.cols();
			msg.layout.dim[1].stride = matrix.cols();
			msg.layout.dim[1].label = "cols";
			msg.data.clear();
			for (int i = 0; i < matrix.rows(); i++) {
				for (int j = 0; j < matrix.cols(); j++) {
					msg.data.push_back(matrix(i, j));
				}
			}
			return msg;
		}

		// Function for eigen matrix (row major) to std_msgs::msg::Float32MultiArray conversion
		std_msgs::msg::Float32MultiArray EigenMatrixRowMajorToFloat32MultiArray(Eigen::MatrixXf matrix) {
			std_msgs::msg::Float32MultiArray msg;
			// Convert eigen matrix to std_msgs::msg::Float32MultiArray
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[0].size = matrix.rows();
			msg.layout.dim[0].stride = matrix.cols() * matrix.rows();
			msg.layout.dim[0].label = "rows";
			msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
			msg.layout.dim[1].size = matrix.cols();
			msg.layout.dim[1].stride = matrix.cols();
			msg.layout.dim[1].label = "cols";
			msg.data.clear();
			msg.data.reserve(matrix.rows() * matrix.cols());
			for (int i = 0; i < matrix.rows(); i++) {
				for (int j = 0; j < matrix.cols(); j++) {
					msg.data.push_back(matrix(i, j));
				}
			}
			return msg;
		}
};
