#!/usr/bin/env python3
import time
import ros_numpy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

import rospkg

rospy.init_node("yolo_segmentation_2d")
time.sleep(1)

input_topic = rospy.get_param('~input_topic', '/usb_cam/image_raw')
class_topic = rospy.get_param('~class_topic', '/yolo/segmentation/classes')
output_topic = rospy.get_param('~output_topic', '/yolo/segmentation/image')

classes_pub = rospy.Publisher(class_topic, String, queue_size=1)
seg_image_pub = rospy.Publisher(output_topic, Image, queue_size=1)

rospack = rospkg.RosPack()
package_path = rospack.get_path('ultralytics_ros')
segmentation_model = YOLO(package_path+"/weights/yolo11m-seg.pt")

def callback(data):
    array = ros_numpy.numpify(data)
    if classes_pub.get_num_connections() or seg_image_pub.get_num_connections():
        seg_result = segmentation_model(array)
        classes = seg_result[0].boxes.cls.cpu().numpy().astype(int)
        names = [seg_result[0].names[i] for i in classes]
        classes_pub.publish(String(data=str(names)))
        seg_annotated = seg_result[0].plot(show=False)
        seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))

    
rospy.Subscriber(input_topic, Image, callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("process killed")
