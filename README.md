# yolo_ros (for ros1)
This package is ROS1 wrap package for realtime object detection and segmentation using [Ultralytics](https://github.com/ultralytics/ultralytics) library. In this package, you can either use default yolov11 model or train your own dataset.
If you use ROS2, [yolo_ros (for ROS2)](https://github.com/mgonzs13/yolo_ros) might be very useful.

## Requirements
- ROS1(noetic for ubuntu20.04, ros-o for ubuntu22.04, ubuntu24.04)

## Installation
Please run the following command in your terminal.
```bash
cd <workspace_dir_path>
source devel/setup.bash
cd src
git clone https://github.com/Michi-Tsubaki/yolo_ros.git
rosdep install --from-path . -i -r -y
cd ..
source devel/setup.bash
catkin build yolo_ros
```

## Simple use
### Detection
```bash
roslaunch yolo_ros yolo_detection_2d.lauch
```
#### option
- `input_topic`: default=`/usb_cam/image_raw`
- `class_topic`: default=`/yolo/detection/classes`
- `output_topic`: default=`/yolo/detection/image`
- `queue`: default=`1`
- `model`: default=`yolo11m.pt`



### Segmentation
```bash
roslaunch yolo_ros yolo_segmentation_2d.lauch
```
#### option
- `input_topic`: default=`/usb_cam/image_raw`
- `class_topic`: default=`/yolo/segmentation/classes`
- `output_topic`: default=`/yolo/segmentation/image`
- `queue`: default=`1`
- `model`: default=`yolo11m-seg.pt`


## Models
If you want to use pre-trained models, you can search avilable models online from following website. Default' yolo11m.pt will be installed when running catkin build.
- [YOLOv3](https://docs.ultralytics.com/models/yolov3/)
- [YOLOv4](https://docs.ultralytics.com/models/yolov4/)
- [YOLOv5](https://docs.ultralytics.com/models/yolov5/)
- [YOLOv6](https://docs.ultralytics.com/models/yolov6/)
- [YOLOv7](https://docs.ultralytics.com/models/yolov7/)
- [YOLOv8](https://docs.ultralytics.com/models/yolov8/)
- [YOLOv9](https://docs.ultralytics.com/models/yolov9/)
- [YOLOv10](https://docs.ultralytics.com/models/yolov10/)
- [YOLOv11](https://docs.ultralytics.com/models/yolo11/)
- [YOLOv12](https://docs.ultralytics.com/models/yolo12/)
- [YOLO-World](https://docs.ultralytics.com/models/yolo-world/)

## Train your own dataset and use the weight
