# Result from DOPE
Once DOPE has detected an image, it returns a message of type `vision_msgs/Detection3DArray` on the topic `/dope/detected_objects`. However, not all data of this message is filled. The normal structure of this message is shown below.
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
vision_msgs/Detection3D[] detections
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  vision_msgs/ObjectHypothesisWithPose[] results
    int64 id
    float64 score
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
  vision_msgs/BoundingBox3D bbox
    geometry_msgs/Pose center
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    geometry_msgs/Vector3 size
      float64 x
      float64 y
      float64 z
  sensor_msgs/PointCloud2 source_cloud
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    uint32 height
    uint32 width
    sensor_msgs/PointField[] fields
      uint8 INT8=1
      uint8 UINT8=2
      uint8 INT16=3
      uint8 UINT16=4
      uint8 INT32=5
      uint8 UINT32=6
      uint8 FLOAT32=7
      uint8 FLOAT64=8
      string name
      uint32 offset
      uint8 datatype
      uint32 count
    bool is_bigendian
    uint32 point_step
    uint32 row_step
    uint8[] data
    bool is_dense
```

As said, not all data is filled by DOPE. The following data is missing:
 - The `source_cloud` data of type `sensor_msgs/PointCloud2`
 - The `covariance` data in `detections/results/pose/covariance` of type `float64[]`

A few other things to notice are:
 - Both header fields `header` and `detections/header` are the same as the incoming image message
 - Both `detections/results/pose` and `detections/bbox/center` are exactly the same pose
 - `detections/results` is given as an array, but will always just contain a single pose
 - For each object that is detected a new item in the `detections` array is added

The data structure of the valid data is shown below. Note that the other fields are still in the message, but are never set by DOPE and should _never_ be used, because the data is not relevant.
 ```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
vision_msgs/Detection3D[] detections
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  vision_msgs/ObjectHypothesisWithPose[] results
    int64 id
    float64 score
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
  vision_msgs/BoundingBox3D bbox
    geometry_msgs/Pose center
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    geometry_msgs/Vector3 size
      float64 x
      float64 y
      float64 z
```

DOPE publishes more messages on different topics, but all that data can also be found in the message above. Note that the two poses that are returned, are relative to the camera. In order to place them in the reference frame of the robot planning the coordinates should be transformed. This can be done with the help of the [tf package](http://wiki.ros.org/tf) :
```python
import tf
from geometry_msgs.msg import PoseStamped

listener = tf.TransformListener()

# The input pose contains position, orientation and a frame_id
# For example, a frame_id of 'camera_sim_link'
input_pose = PoseStamped()
input_pose.header.frame_id = 'camera_sim_link'

# output_pose contains the same pose, but in the reference frame of the base_link
output_pose = listener.transformPose('base_link', input_pose)
```

Read next:  
[Analyizing dope training](Analyzing%20DOPE%20training.md)