#include <ignition/msgs/pose_v.pb.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class SimDataPublisher : public rclcpp::Node {

	std::shared_ptr<ignition::transport::Node> mIgnNode;
  	std::shared_ptr<diagnostic_updater::Updater> mUpdater;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPublisher;
	std::string mRobotId;
	bool mPublishTf;
	std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
	std::unique_ptr<tf2_ros::TransformListener> mTfListener;
	std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

	double mStatsRealTimeFactor = 0.;
	uint64_t mStatsIterations = 0;
	uint64_t mStatsSimTimeNS = 0;
	uint64_t mStatsRealTimeNS = 0;

	void dynamicPoseCallback(const ignition::msgs::Pose_V &Poses);
	void statsCallback(const ignition::msgs::WorldStatistics &worldStats);

public:
	SimDataPublisher();
	void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
};


SimDataPublisher::SimDataPublisher() : Node("sim_groundtruth_publisher") {

	declare_parameter("robot_id", "unknown");
	declare_parameter("world_name", "default");
	declare_parameter("publish_tf", false);

	mUpdater = std::make_shared<diagnostic_updater::Updater>(this);
	mUpdater->setHardwareID("simulation");

	get_parameter("robot_id", mRobotId);
	get_parameter("publish_tf", mPublishTf);

	if (mPublishTf) {
		RCLCPP_INFO(get_logger(),"Publishing map->odom tf...");
		mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
		mTfListener = std::make_unique<tf2_ros::TransformListener>(*mTfBuffer);
		mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	}

	mPublisher = create_publisher<geometry_msgs::msg::PoseStamped>(
		"/groundtruth_pose", 10);

  	mIgnNode = std::make_shared<ignition::transport::Node>();
  	std::string world_name;
  	get_parameter("world_name", world_name);

	mIgnNode->Subscribe("/world/" + world_name + "/dynamic_pose/info",
						&SimDataPublisher::dynamicPoseCallback, this);

	mIgnNode->Subscribe("/world/" + world_name + "/stats",
						&SimDataPublisher::statsCallback, this);
						
	mUpdater->add("SimDataPublisher", this, &SimDataPublisher::produce_diagnostics);
}


void SimDataPublisher::dynamicPoseCallback(const ignition::msgs::Pose_V &Poses) {
  	auto now = get_clock()->now();
  	for (int i = 0; i < Poses.pose_size(); i++) {
    	const ignition::msgs::Pose *pp = &Poses.pose(i);
    	if (pp->name() == mRobotId) {
			const ignition::msgs::Vector3d *pv = &pp->position();
			const ignition::msgs::Quaternion *pq = &pp->orientation();

			geometry_msgs::msg::PoseStamped pcs;
			pcs.header.stamp = now;
			pcs.header.frame_id = "world";
			pcs.pose.position.x = pv->x();
			pcs.pose.position.y = pv->y();
			pcs.pose.position.z = pv->z();
			pcs.pose.orientation.x = pq->x();
			pcs.pose.orientation.y = pq->y();
			pcs.pose.orientation.z = pq->z();
			pcs.pose.orientation.w = pq->w();
			mPublisher->publish(pcs);

			// calculate and broadcast the map->odom TF
			if(mPublishTf) {
				try {
					geometry_msgs::msg::TransformStamped transform;
					transform.header.stamp=now;
					transform.transform.translation.x=pv->x();
					transform.transform.translation.y=pv->y();
					transform.transform.translation.z=pv->z();
					transform.transform.rotation.x=pq->x();
					transform.transform.rotation.y=pq->y();
					transform.transform.rotation.z=pq->z();
					transform.transform.rotation.w=pq->w();

					geometry_msgs::msg::TransformStamped baseToOdom = mTfBuffer->lookupTransform("base_link", "odom", tf2::TimePointZero);
					geometry_msgs::msg::TransformStamped mapToOdom;
					tf2::doTransform(baseToOdom, mapToOdom, transform);

					mapToOdom.header.frame_id = "map";
					mapToOdom.header.stamp = baseToOdom.header.stamp;
					mapToOdom.child_frame_id = "odom";
					mTfBroadcaster->sendTransform(mapToOdom);
				} catch (tf2::TransformException &ex) {
					RCLCPP_WARN(get_logger(),"Transformation Exception in sim_data_publisher: %s",ex.what());
				}
			}
    	}
  	}
}


void SimDataPublisher::statsCallback(const ignition::msgs::WorldStatistics &worldStats) {

	mStatsSimTimeNS = worldStats.sim_time().sec() * 1000000000 + worldStats.sim_time().nsec();
	mStatsRealTimeNS = worldStats.real_time().sec() * 1000000000 + worldStats.real_time().nsec();
	mStatsRealTimeFactor = worldStats.real_time_factor();
	mStatsIterations = worldStats.iterations();
}


void SimDataPublisher::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
	stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
	stat.add("real_time_factor", mStatsRealTimeFactor);
	stat.add("iterations", mStatsIterations);
	stat.add("real_time_ns", mStatsRealTimeNS);
	stat.add("sim_time_ns", mStatsSimTimeNS);
}


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimDataPublisher>());
	rclcpp::shutdown();

	return 0;
}
