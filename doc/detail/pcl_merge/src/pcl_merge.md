# Point Cloud Merge Implementation (src/pcl_merge/src/pcl_merge.cpp)

## Overview

This file implements multi-sensor point cloud fusion with TF2 transformation and voxel grid downsampling. It subscribes to multiple point cloud topics, transforms them to a common frame, and publishes a merged, downsampled cloud.

## Class: PCLMergeNode

### Constructor

**Location:** Lines 3-40

**Key Operations:**

1. **Parameter Declaration** (Lines 8-12)
```cpp
this->declare_parameter("output_frame", "base_link");
this->declare_parameter("output_topic", "pcl_merged");
this->declare_parameter("input_topics",
    std::vector<std::string>({"scan_pointcloud",
                               "/camera_depth_top/camera_depth/points"}));
```

2. **Dynamic Subscription** (Lines 26-29)
```cpp
for (size_t i = 0; i < input_topics_.size(); i++) {
    auto callback = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(i, msg);
    };
    subscribers_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i], 10, callback));
}
```

**Lambda captures:** Index `i` bound to each callback for cloud storage

3. **TF2 Setup** (Lines 35-36)
```cpp
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

4. **Timer Configuration** (Line 39)
```cpp
timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                  std::bind(&PCLMergeNode::timer_callback, this));
```
Publishes at ~30 Hz (33ms period)

## Point Cloud Callback

### cloud_callback()

**Location:** Lines 42-116

**Purpose:** Receive, transform, and store point clouds from individual sensors

### Processing Pipeline

#### 1. Index Validation (Lines 43-44)
```cpp
if (index >= clouds_.size())
    return;
```

#### 2. TF Transform Lookup (Lines 47-56)
```cpp
transform = tf_buffer_->lookupTransform(
    output_frame_,              // Target: base_link
    msg->header.frame_id,       // Source: sensor frame
    rclcpp::Time(0)             // Latest available
);
```

**Error handling:** Warns and discards cloud if transform unavailable

#### 3. Point Type Conversion (Lines 58-100)

**Strategy:** Try direct PointXYZI conversion, fallback to PointXYZRGB

**Direct conversion attempt:**
```cpp
try {
    pcl::fromROSMsg(*msg, *pcl_cloud);
    conversion_success = true;
} catch (std::exception &e) {
    // Try RGB conversion
}
```

**RGB to Intensity conversion:**
```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(...);
pcl::fromROSMsg(*msg, *rgb_cloud);

for (size_t i = 0; i < rgb_cloud->points.size(); ++i) {
    // Extract RGB values
    uint32_t rgb = *reinterpret_cast<const int*>(&pt.rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;

    // Luminosity formula: I = 0.299*R + 0.587*G + 0.114*B
    pt_i.intensity = 0.299 * r + 0.587 * g + 0.114 * b;
}
```

**Rationale:** ITU-R BT.601 luma coefficients for perceptual brightness

#### 4. Spatial Transformation (Lines 103-112)
```cpp
pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed, transform);
```

Applies TF2 transform to align all clouds to `output_frame`

#### 5. Storage (Line 115)
```cpp
clouds_[index] = pcl_cloud_transformed;
```

Each sensor's latest cloud stored separately

## Merging and Publishing

### timer_callback()

**Location:** Lines 119-156

**Purpose:** Periodically merge all clouds and publish downsampled result

### Processing Steps

#### 1. Validity Check (Lines 121-130)
```cpp
bool any_valid = false;
for (const auto &cloud : clouds_) {
    if (cloud != nullptr) {
        any_valid = true;
        break;
    }
}
if (!any_valid) return;
```

Only proceeds if at least one cloud available

#### 2. Cloud Merging (Lines 133-138)
```cpp
pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(...);
for (const auto& cloud : clouds_) {
    if (cloud) {
        *merged_cloud += *cloud;  // PCL cloud concatenation
    }
}
```

**Operator overload:** PCL's `operator+=` concatenates point clouds

#### 3. Voxel Grid Downsampling (Lines 141-146)
```cpp
pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
voxel_filter.setInputCloud(merged_cloud);
voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // 5cm cubes
voxel_filter.filter(*downsampled_cloud);
```

**Effect:** Replaces all points within each 5cm³ voxel with their centroid

**Benefits:**
- Reduces point count (typically 5-10x)
- Removes duplicate points from overlapping sensors
- Regularizes point spacing

#### 4. ROS Message Conversion (Lines 149-151)
```cpp
sensor_msgs::msg::PointCloud2 output_msg;
pcl::toROSMsg(*downsampled_cloud, output_msg);
output_msg.header.frame_id = output_frame_;
```

#### 5. Publishing (Line 154)
```cpp
pub_->publish(output_msg);
RCLCPP_INFO_ONCE(...);  // Log first publish only
```

## Main Function

**Location:** Lines 159-166

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLMergeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Data Flow Diagram

```
Sensor 1 → cloud_callback(0) → Transform → Store[0] ─┐
Sensor 2 → cloud_callback(1) → Transform → Store[1] ─┤
Sensor N → cloud_callback(N) → Transform → Store[N] ─┴→ timer_callback()
                                                         ↓
                                                    Merge clouds
                                                         ↓
                                                    Voxel downsample
                                                         ↓
                                                    Publish merged cloud
```

## Memory Management

- **Cloud storage:** `std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>`
- **Overwrite policy:** Latest cloud from each sensor replaces previous
- **Temporary clouds:** Created each timer cycle, automatically destroyed
- **Smart pointers:** RAII ensures proper cleanup

## Performance Optimization

### Efficient Type Handling
- Direct PointXYZI conversion when possible
- RGB conversion only as fallback
- Avoids unnecessary memory copies

### Voxel Grid Benefits
- **Input:** Potentially millions of points
- **Output:** Typically 10-100k points
- **Speed:** O(n) filtering
- **Memory:** Significant reduction

### Timer-Based Publishing
- Decouples input rates from output rate
- Constant 30 Hz output regardless of sensor rates
- Asynchronous processing

## Error Scenarios

| Error | Handling | Impact |
|-------|----------|--------|
| Missing TF transform | Warn and discard | Cloud not merged this cycle |
| Invalid point type | Try RGB conversion | Graceful fallback |
| RGB conversion fails | Warn and discard | Sensor excluded |
| No valid clouds | Silent return | No output published |

## Configuration Recommendations

### Voxel Leaf Size Tuning
- **0.01m:** High detail, slower processing
- **0.05m:** Balanced (default)
- **0.10m:** Fast, lower detail

### Output Frame Selection
- **base_link:** Robot-centric (recommended)
- **map:** Global frame
- **odom:** For filtered odometry

## Dependencies

- **PCL:** Point cloud structures and filters
- **PCL Conversions:** ROS ↔ PCL conversion
- **PCL ROS:** TF integration
- **TF2:** Transform management
- **Eigen3:** Matrix math (via PCL)
