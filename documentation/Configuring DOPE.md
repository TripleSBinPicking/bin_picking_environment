# Configuring DOPE
DOPE can be configured to detect different objects. This document will describe how this must be done.

## Configuration file
The configuration file is formatted in [YAML](https://en.wikipedia.org/wiki/YAML). The default configuration that is loaded is located in [`triple_s_util/config/dope.yaml`](../triple_s_util/config/dope.yaml).

### Basic settings
The following settings are some basic settings that configure DOPE.

| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `topic_camera` | ROS topic name | `/dope/webcam/image_raw` | The topic that the camera publishes images on |
| `topic_camera_info` | ROS topic name | `/dope/webcam/camera_info` | The topic the camera publishes it's camera intrinsics on. For every message on `topic_camera` a message should also be sent on `topic_camera_info` |
| `topic_publishing` | string | `dope` | The prefix for the ROS topics that DOPE publishes results on |
| `input_is_rectified` | boolean | `True` | If true the image will be rectified for the camera distortion |
| `downscale_height` | integer | `500` | Scale the input image to this heigt (larger images take more processing power) |

### Object settings
The following settings are used to detect the position of objects. Each parameter is a list. The key of the list is always the name of the object that will be used. The values that can be used are explained in the table below. An example usage is also given below.

```yaml
weights: {
    "<Object name>": <value>,
}
```

| Parameter | Valid values | Explanation |
| --- | --- | --- |
| `weights` | File path to weights of object | The weights of an object are in the `.pth` file that was the result of a trainings session |
| `dimensions` | Array of size three (`[5, 2, 3]`) | Object cuboid dimensions in cm (x, y, z) |
| `class_ids` | integer | The ids are used internally to identify the objects. Each id should be unique for each object. |
| `draw_colors` | Array of size three (`[5, 2, 3]`) | The color of the line that will be drawn on the image if an detection occurs for this object. |
| `meshes` | File path to object mesh (optional) | If a mesh path is given the mesh of the object is shown in Rviz instead of a cuboid |
| `mesh_scales` | float | The scale of the mesh to meters. (So 1 if the mesh is in meters) |

### Inference settings
The final few settings are for the AI itself and how it determines where objects are. Sadly they are not documented by DOPE.

| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `thresh_angle` | float | `0.5` | Undocumented |
| `thresh_map` | float | `0.01` | Undocumented |
| `sigma` | float | `3` | Undocumented |
| `thresh_points` | float | `0.1` | The minimum confidence DOPE must have for a detection in order to return the result |

## Adding new objects
In order to add new objects, you first have to [create a dataset for the object](NDDS%20tutorial.md) and [train on that dataset](Training%20DOPE.md). As a result of the training you will get a so-called weights file for the object (the `.pth` file from the training). If you don't have the resources to train the AI yourself, you can use some pre-trained models provided by DOPE. They can be downloaded [here](https://drive.google.com/drive/folders/1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg). The 3D models are also available for these objects and they are located [here](https://drive.google.com/drive/folders/1jiJS9KgcYAkfb8KJPp5MRlB0P11BStft).

Once the weights file is ready, you have to configure at least `weights`, `dimensions`, `class_ids` and `draw_colors` in your configuration file.

Read next:
[Picking up an object](Picking%20up%20object%20overview.md)