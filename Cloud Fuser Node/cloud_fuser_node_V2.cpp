// cloud_fuser_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

using std::placeholders::_1;
using std::placeholders::_2;

class PointCloudFuser : public rclcpp::Node {
public:
    PointCloudFuser()
        : Node("pointcloud_fuser"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter("z_min", 0.05);
        this->declare_parameter("z_max", 0.4);
        this->declare_parameter("voxel_size", 0.05);

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_points", 10);

        sub_rgbd_.subscribe(this, "/camera/camera/depth/color/points");
        sub_livox_.subscribe(this, "/livox/lidar");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), sub_rgbd_, sub_livox_);
        sync_->registerCallback(std::bind(&PointCloudFuser::callback, this, _1, _2));
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2>;

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_rgbd,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_livox)
    {
        try {
            auto transform_rgbd = tf_buffer_.lookupTransform("base_link", cloud_rgbd->header.frame_id, tf2::TimePointZero);
            auto transform_livox = tf_buffer_.lookupTransform("base_link", cloud_livox->header.frame_id, tf2::TimePointZero);

            sensor_msgs::msg::PointCloud2 cloud_rgbd_trans, cloud_livox_trans;
            tf2::doTransform(*cloud_rgbd, cloud_rgbd_trans, transform_rgbd);
            tf2::doTransform(*cloud_livox, cloud_livox_trans, transform_livox);

            pcl::PointCloud<pcl::PointXYZ>::Ptr rgbd_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr livox_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(cloud_rgbd_trans, *rgbd_cloud);
            pcl::fromROSMsg(cloud_livox_trans, *livox_cloud);

            auto z_min = this->get_parameter("z_min").as_double();
            auto z_max = this->get_parameter("z_max").as_double();
            auto voxel_size = this->get_parameter("voxel_size").as_double();

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min, z_max);
            pass.setInputCloud(rgbd_cloud);
            pass.filter(*rgbd_cloud);
            pass.setInputCloud(livox_cloud);
            pass.filter(*livox_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr combined(new pcl::PointCloud<pcl::PointXYZ>());
            *combined += *rgbd_cloud;
            *combined += *livox_cloud;

            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(combined);
            voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
            voxel.filter(*combined);

            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*combined, output);
            output.header.frame_id = "base_link";
            output.header.stamp = this->get_clock()->now();

            pub_->publish(output);

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Fusion failed: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_rgbd_, sub_livox_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFuser>());
    rclcpp::shutdown();
    return 0;
}
