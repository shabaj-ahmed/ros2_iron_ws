#include <cstring>  // std::memcpy
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace smsg = sensor_msgs::msg;

class HeightFilterNode : public rclcpp::Node {
public:
  HeightFilterNode() : rclcpp::Node("height_filter_node") {
    // Parameters read once at startup (no dynamic updates).
    min_height_ = this->declare_parameter<double>("min_height", -1.0);
    max_height_ = this->declare_parameter<double>("max_height",  2.0);

    // QoS suited for sensor streams (LiDAR).
    const auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<smsg::PointCloud2>(
      /* topic    = */ "input",
      /* qos      = */ qos,
      /* callback = */ std::bind(&HeightFilterNode::cloudCallback, this, std::placeholders::_1)
    );

    // Publisher for filtered output.
    pub_ = this->create_publisher<smsg::PointCloud2>(
      /* topic = */ "output",
      /* qos   = */ qos
    );

    RCLCPP_INFO(
      this->get_logger(),
      "HeightFilterNode started. min_height=%.3f, max_height=%.3f",
      min_height_, max_height_
    );
  }

private:
  void cloudCallback(const smsg::PointCloud2::SharedPtr msg) {
    // First pass: count how many points pass the height filter (z in [min, max])
    std::size_t keep = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z"); z != z.end(); ++z) {
      if (*z >= min_height_ && *z <= max_height_) ++keep;
    }

    // Prepare output: same layout, just fewer points
    smsg::PointCloud2 out;
    out.header       = msg->header;
    out.fields       = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step   = msg->point_step;
    out.height       = 1;
    out.width        = static_cast<uint32_t>(keep);
    out.row_step     = out.point_step * out.width;
    out.is_dense     = false;
    out.data.resize(out.row_step);

    // Second pass: copy passing points byte-for-byte using point_step
    size_t in_index = 0;   // index of current input point
    size_t out_index = 0;  // index of next output point to write

    // Iterate with z to step through points
    sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

    for (; z_it != z_it.end(); ++z_it, ++in_index) {
      float z = *z_it;
      if (z >= min_height_ && z <= max_height_) {
        const size_t in_offset  = in_index * msg->point_step;
        const size_t out_offset = out_index * out.point_step;
        std::memcpy(&out.data[out_offset], &msg->data[in_offset], msg->point_step);
        ++out_index;
      }
    }
    
    pub_->publish(std::move(out));
  }

  rclcpp::Subscription<smsg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<smsg::PointCloud2>::SharedPtr    pub_;
  double min_height_;
  double max_height_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeightFilterNode>());
  rclcpp::shutdown();
  return 0;
}
