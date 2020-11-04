# Pose estimation video
This document describes how to create a video with the pose estimation of the objects drawn on it.

### Step 1: Create image sequence and camera config
To be able to create a video with the pose estimation in it, it is necessary to evaluate every frame individually. It is also necessary to create a file which describes the camera. This can both be automatically done.
```bash
roslaunch triple_s_util video_to_images
```
By default the program will use the webcam (only works on laptops). If you whish to use your own topics you can set them using the following parameters:
 - `camera_raw_topic` the topic on which the `sensor_msgs/Image` messages are published
 - `camera_info_topic` the topic on which the `sensor_msgs/CameraInfo` messages are published
 - `start_webcam` must be set to `false` to disable the webcam

Other parameters are:
 - `path` the path to which the images should be saved (defaults to `local_resources/raw_images`)
 - `total_images` the total amount of images that must be saved (defaults to `150` for 5 seconds of video at 30 fps)
 - `camera_info_output` the name of the file with the camera information (defaults to `camera_info.yaml`)

After running the launchfile the camera will start recording. The images will be placed in the location defined by `path`. The camera will automatically stop recording after the set amount of images are saved. The images are named `image{xxxx}.jpeg` were `{xxxx}` is replaced with the four-digit frame number.

### Step 2: Send the images through DOPE
Once the images are ready it is time to let DOPE analyse them. This can be done by executing the following command:
```bash
roslaunch triple_s_util images_to_dope.launch
```

This will analyse the images placed in the `local_resources/raw_images` and publish the results in `local_resources/dope_images`. Both are configurable with parameters (see the launchfile for more details).

After it is done analyzing al the images it will print `Finished!` in the commandline and you can exit the program.

### Step 3: Creating a video from the images
DOPE created another set with annotated images, but this isn't a nice video yet. To make a video from the images we use `ffmpeg` (`apt install ffmpeg`).
Go to the directory with the DOPE images (`local_resources/dope_images`) and execute the following command to create a `mp4`-video file:
```bash
ffmpeg -framerate 30 -i image%04d.jpeg output.mp4
```
You can set the framerate according to your footage.
