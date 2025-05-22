#!/usr/bin/env python3
import time
import ros_numpy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

segmentation_model = YOLO("../weights/yolo11m-seg.pt")
rospy.init_node("yolo_segmentation_2d")
time.sleep(1)

classes_pub = rospy.Publisher("/yolo/segmentation/classes", String, queue_size=1)
seg_image_pub = rospy.Publisher("/yolo/segmentation/image", Image, queue_size=1)

def callback(data):
    """Callback function to process image and publish detected classes."""
    array = ros_numpy.numpify(data)
    if classes_pub.get_num_connections() and seg_image_pub.get_num_connections():
        seg_result = segmentation_model(array)
        classes = seg_result[0].boxes.cls.cpu().numpy().astype(int)
        names = [seg_result[0].names[i] for i in classes]
        classes_pub.publish(String(data=str(names)))
        seg_annotated = seg_result[0].plot(show=False)
        det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))

rospy.Subscriber("/usb_cam/image_raw", Image, callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("process killed")
