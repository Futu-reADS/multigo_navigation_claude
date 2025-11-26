# ArUco Detection Implementation (src/aruco_detect/src/aruco_detect.cpp)

## Overview

This file implements the core ArUco marker detection node using OpenCV's ArUco module. It subscribes to camera topics, detects markers, estimates their 6-DOF poses, and publishes the results.

## Class: Aruco_detect

### Constructor

**Location:** Lines 5-40

**Functionality:**
- Declares and retrieves ROS2 parameters
- Creates image and camera info subscriptions
- Initializes pose array publisher
- Sets up TF2 broadcaster for marker transforms
- Creates OpenCV visualization window

**Key Parameters:**
- `desired_aruco_marker_id`: Which marker to detect (default: 0)
- `marker_width`: Physical marker size in meters (default: 0.05m)
- `camera_topic`: Image input topic
- `camera_info`: Camera calibration topic

**Dependencies:**
```cpp
#include "aruco_detect/aruco_detect.h"
```

## Camera Calibration Callback

### cameraInfoCallback()

**Location:** Lines 43-80

**Purpose:** Store camera intrinsic parameters for pose estimation

**Process:**
1. Check if calibration already received (one-time operation)
2. Extract frame_id and image dimensions
3. Parse camera matrix (K) from 9-element array
4. Parse distortion coefficients (D) from 5-element array
5. Parse rectification matrix (R)
6. Parse projection matrix (P)
7. Set `cam_info_received` flag

**Storage Format:**
```cpp
camera_matrix = (cv::Mat_<double>(3, 3) <<
    K[0], K[1], K[2],
    K[3], K[4], K[5],
    K[6], K[7], K[8]);
```

## Image Processing Callback

### imageCallback()

**Location:** Lines 83-104

**Purpose:** Process incoming camera images for marker detection

**Process:**
1. Wait for camera calibration to be received
2. Refresh parameters (allows runtime reconfiguration)
3. Convert ROS Image to OpenCV format (MONO8)
4. Handle cv_bridge exceptions
5. Call marker detection function

**Error Handling:**
```cpp
try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
} catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
}
```

## Marker Detection and Pose Estimation

### detectArucoMarkers()

**Location:** Lines 106-243

**Purpose:** Main detection and pose estimation algorithm

### Algorithm Steps

#### 1. Initialization (Lines 108-114)
```cpp
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners;
std::vector<cv::Vec3d> rvecs, tvecs;
```

#### 2. Parameter Validation (Lines 121-125)
Ensures camera matrix and distortion coefficients are initialized

#### 3. Marker Detection (Line 128)
```cpp
cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
```

Uses predefined ArUco dictionary (DICT_4X4_50)

#### 4. Pose Estimation (Line 134)
```cpp
cv::aruco::estimatePoseSingleMarkers(
    markerCorners,    // Detected corners
    marker_width,     // Physical marker size
    camera_matrix,    // Intrinsic matrix
    dist_coeffs,      // Distortion coefficients
    rvecs,           // Output: rotation vectors
    tvecs            // Output: translation vectors
);
```

#### 5. Marker Filtering (Lines 137-139)
Only processes markers matching `desired_aruco_marker_id`

#### 6. Coordinate Transformation (Lines 145-184)

**OpenCV to ROS Conversion:**

**Position mapping:**
```cpp
pose.position.x = tvecs[i][2];   // Forward (OpenCV Z → ROS X)
pose.position.y = -tvecs[i][0];  // Left (OpenCV -X → ROS Y)
pose.position.z = tvecs[i][1];   // Up (OpenCV Y → ROS Z)
```

**Rotation conversion:**
```cpp
// Step 1: Rodrigues rotation vector → rotation matrix
cv::Rodrigues(rvecs[i], rotation_matrix);

// Step 2: Convert to Eigen format
Eigen::Matrix3d rot = ... // Copy rotation_matrix

// Step 3: Apply coordinate frame transformation
Eigen::Matrix3d cv_to_ros;
cv_to_ros << 0,  0, 1,
            -1,  0, 0,
             0, -1, 0;
rot = cv_to_ros * rot * cv_to_ros.transpose();

// Step 4: Convert to quaternion
Eigen::Quaterniond eigen_quat(rot);
tf2::Quaternion quat(eigen_quat.x(), eigen_quat.y(),
                     eigen_quat.z(), eigen_quat.w());

// Step 5: Apply 180° pitch offset
tf2::Quaternion offset;
offset.setRPY(0, M_PI, 0);
quat = quat * offset;
quat.normalize();
```

#### 7. Visualization (Lines 186-195)
Display marker info on image:
```cpp
cv::putText(image, position_text,
            cv::Point(10, 80),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5, cv::Scalar(255, 0, 0), 1);
```

#### 8. TF Broadcasting (Lines 200-212)
```cpp
transformStamped.header.stamp = current_time;
transformStamped.header.frame_id = frame_id;  // Camera frame
transformStamped.child_frame_id = "aruco_marker_" + std::to_string(markerIds[i]);
// ... set translation and rotation
tf_broadcaster->sendTransform(transformStamped);
```

#### 9. Pose Array Publishing (Lines 213-218)
Add pose to array and publish

#### 10. FPS Calculation (Lines 225-238)
Weighted moving average:
```cpp
float duration = (duration_fps.seconds() +
                  (previous_duration * (weighted_avg-1))) / weighted_avg;
fps = 1.0f / duration;
```

## Main Function

**Location:** Lines 247-253

**Functionality:**
```cpp
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aruco_detect::Aruco_detect>());
    rclcpp::shutdown();
    return 0;
}
```

## Coordinate Frame Convention

### OpenCV Camera Frame
- **X-axis:** Right
- **Y-axis:** Down
- **Z-axis:** Forward (optical axis)

### ROS Camera Frame
- **X-axis:** Forward
- **Y-axis:** Left
- **Z-axis:** Up

### Transformation Matrix
```
[0   0  1]   [X_cv]   [Z_cv]      [X_ros]
[-1  0  0] × [Y_cv] = [-X_cv]  =  [Y_ros]
[0  -1  0]   [Z_cv]   [-Y_cv]     [Z_ros]
```

## Dependencies

### ROS2
- `rclcpp`: Node management
- `sensor_msgs`: Image and CameraInfo messages
- `geometry_msgs`: Pose and Transform messages
- `tf2_ros`: Transform broadcasting
- `cv_bridge`: ROS-OpenCV conversion

### External
- **OpenCV 4.x:** ArUco detection
- **Eigen3:** Matrix operations
- **TF2:** Quaternion math

## Performance Considerations

- **FPS display:** Real-time performance monitoring
- **One-time calibration:** Camera parameters loaded once
- **Selective detection:** Only processes desired marker ID
- **Efficient conversion:** Direct memory mapping where possible

## Error Handling

1. **Missing calibration:** Returns early from imageCallback
2. **cv_bridge failure:** Logs error and returns
3. **No markers:** Silent (avoids log spam)
4. **Uninitialized parameters:** Check before use

## Future Enhancements

- [ ] Multi-marker tracking
- [ ] Marker pose filtering (Kalman filter)
- [ ] Dynamic marker size parameter
- [ ] Support for multiple ArUco dictionaries
- [ ] Marker detection confidence thresholding
