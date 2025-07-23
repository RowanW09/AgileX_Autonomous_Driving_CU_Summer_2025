// cloud_fuser_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/filters/statistical_outlier_removal.h>


using std::placeholders::_1;
using std::placeholders::_2;

class PointCloudFuser : public rclcpp::Node {
public:
  PointCloudFuser()
  : Node("cloud_fuser_V3"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    rgbd_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
    livox_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
    combined_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
  {
    // --- parameters ---
    this->declare_parameter("z_min", 0.05);
    this->declare_parameter("z_max", 0.4);
    this->declare_parameter("voxel_size", 0.05);
    updateFiltersFromParams();

    // allow dynamic updates
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudFuser::onParamUpdate, this, _1)
    );

    // reserve to avoid reallocations
    const size_t MAX_POINTS = 200000;
    rgbd_cloud_->points.reserve(MAX_POINTS);
    livox_cloud_->points.reserve(MAX_POINTS);
    combined_cloud_->points.reserve(MAX_POINTS * 2);

    // publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_points", 10);

    // subscribers + sync
    sub_rgbd_.subscribe(this, "/camera/camera/depth/color/points");
    sub_livox_.subscribe(this, "/livox/lidar");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_rgbd_, sub_livox_);
    sync_->registerCallback(std::bind(&PointCloudFuser::callback, this, _1, _2));
  }

private:
  using Cloud2 = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Cloud2, Cloud2>;

  // PCL filters (reused)
  pcl::PassThrough<pcl::PointXYZ> pass_;
  pcl::VoxelGrid<pcl::PointXYZ>   voxel_;

  // reusable clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr livox_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_;

  // temp ROS msgs
  Cloud2 tmp_rgbd_msg_, tmp_livox_msg_, tmp_output_msg_;

  // cached parameter values
  double z_min_, z_max_, voxel_size_;

  // ROS interfaces
  rclcpp::Publisher<Cloud2>::SharedPtr pub_;
  message_filters::Subscriber<Cloud2> sub_rgbd_, sub_livox_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;


  // update PCL filters from params
  void updateFiltersFromParams() {
    z_min_      = this->get_parameter("z_min")    .as_double();
    z_max_      = this->get_parameter("z_max")    .as_double();
    voxel_size_ = this->get_parameter("voxel_size").as_double();

    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(z_min_, z_max_);

    voxel_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  }

  // parameter-change callback
  rcl_interfaces::msg::SetParametersResult onParamUpdate(
      const std::vector<rclcpp::Parameter> &params)
  {
    for (auto &p : params) {
      if (p.get_name()=="z_min" || p.get_name()=="z_max" || p.get_name()=="voxel_size") {
        // we'll refresh all three below
      }
    }
    updateFiltersFromParams();
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  // fused-callback
void callback(const Cloud2::ConstSharedPtr in_rgbd,
              const Cloud2::ConstSharedPtr in_livox)
{
  try {
    // start total timer
    rclcpp::Time t_total = this->now();
    rclcpp::Time t_stage = t_total;

    // 1) TF lookup
    auto tf_stamped = tf_buffer_.lookupTransform(
        "base_link", in_rgbd->header.frame_id, tf2::TimePointZero);
    RCLCPP_DEBUG(get_logger(),
      "1) TF lookup: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 2) inline Eigen transform
    Eigen::Affine3d eigen_tf = tf2::transformToEigen(tf_stamped);
    Eigen::Matrix4f mat = eigen_tf.matrix().cast<float>();
    RCLCPP_DEBUG(get_logger(),
      "2) transformToEigen: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 3) ROS→PCL conversion
    pcl::fromROSMsg(*in_rgbd,  *rgbd_cloud_);
    pcl::fromROSMsg(*in_livox, *livox_cloud_);
    RCLCPP_DEBUG(get_logger(),
      "3) fromROSMsg: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 4) PCL transform
    pcl::transformPointCloud(*rgbd_cloud_,  *rgbd_cloud_,  mat);
    pcl::transformPointCloud(*livox_cloud_, *livox_cloud_, mat);
    RCLCPP_DEBUG(get_logger(),
      "4) pcl::transformPointCloud: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 5) passthrough filters
    pass_.setInputCloud(rgbd_cloud_);  pass_.filter(*rgbd_cloud_);
    
    // remove isolated noise
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (rgbd_cloud_);
    sor.setMeanK      (50);       // look at each point’s 50 nearest neighbors
    sor.setStddevMulThresh (1.0); // anything farther than 1×std dev is dropped
    sor.filter (*rgbd_cloud_);
    
    pass_.setInputCloud(livox_cloud_); pass_.filter(*livox_cloud_);
    RCLCPP_DEBUG(get_logger(),
      "5) passthrough: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 6) merge & voxel
    combined_cloud_->clear();
    *combined_cloud_ += *rgbd_cloud_;
    *combined_cloud_ += *livox_cloud_;
    voxel_.setInputCloud(combined_cloud_);
    voxel_.filter(*combined_cloud_);
    RCLCPP_DEBUG(get_logger(),
      "6) merge+voxel: %.1f ms", (this->now() - t_stage).seconds() * 1e3);
    t_stage = this->now();

    // 7) PCL→ROS & publish
    pcl::toROSMsg(*combined_cloud_, tmp_output_msg_);
    tmp_output_msg_.header.frame_id = "base_link";
    tmp_output_msg_.header.stamp = this->now();
    pub_->publish(tmp_output_msg_);
    RCLCPP_DEBUG(get_logger(),
      "7) toROSMsg+publish: %.1f ms", (this->now() - t_stage).seconds() * 1e3);

    // total
    RCLCPP_DEBUG(get_logger(),
      "Total callback: %.1f ms", (this->now() - t_total).seconds() * 1e3);

  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Fusion failed: %s", e.what());
  }
}
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFuser>());
  rclcpp::shutdown();
  return 0;
}
