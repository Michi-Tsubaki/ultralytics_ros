#!/usr/bin/env python3
import time
import ros_numpy
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

detection_model = YOLO("../weights/yolo11m.pt")
rospy.init_node("yolo_detection_2d")
time.sleep(1)
classes_pub = rospy.Publisher("/yolo/detection/classes", String, queue_size=1)
det_image_pub = rospy.Publisher("/yolo/detection/image", Image, queue_size=1)

def callback(data):
    """Callback function to process image and publish detected classes."""
    array = ros_numpy.numpify(data)
    if classes_pub.get_num_connections() or det_image_pub.get_num_connections():
        det_result = detection_model(array)
        classes = det_result[0].boxes.cls.cpu().numpy().astype(int)
        names = [det_result[0].names[i] for i in classes]
        classes_pub.publish(String(data=str(names)))
        det_annotated = det_result[0].plot(show=False)
        det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))

rospy.Subscriber("/usb_cam/image_raw", Image, callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("process killed")
