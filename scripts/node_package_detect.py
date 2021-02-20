#!/usr/bin/env python
"""
This script handles the detection of the colour of the package under the camera.
"""
import time
from datetime import datetime
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pyzbar.pyzbar import decode
from node_iot_action_client import RosIotBridgeActionClient


def get_qr_data(arg_image):
    """
    Obtains data from the QR Code on the package. 
    Returns a result if a QR code is found.

    :arg arg_image: The image to extract data from.
    """
    qr_result = decode(arg_image)

    if qr_result:
        return qr_result[0].data

    return 'NA'


class Camera1(object):
    """
    Camera class to detect on-shelf packages.

    :var int counter: Counts the number of packages detected.
    :var list(str) package_colour: Lists all the packages detected.
    :var frame: Frame for object detection.
    :var bridge: OpenCV bridge variable.
    :var rospy.Subscriber image_sub: Subscriber for the shelf camera image data.
    """
    def __init__(self):
        """
        Constructor containing essential data.
        """
        self.counter = 0
        self.package_colour = []
        self.frame = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)

    def callback(self, data):
        """
        Callback function for :attr:`image_sub <Camera1>`.

        :arg data: The raw image data.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = cv_image
            self.frame = image

        except CvBridgeError as exception:
            rospy.logerr(exception)

    def calculate(self):
        """
        Performs Img. Proc. calculations.
        """
        if self.frame is None:
            return

        box_size = 130
        self.package_colour = []

        first_row_x = 115
        first_row_y = 300

        inter_box_x = 180
        inter_box_y = 180

        offset = 35

        for i in range(3):
            for j in range(3):
                if self.counter == 9:
                    quit()
                if i == 2:
                    img = self.frame[
                        (first_row_y + i * inter_box_y - offset):
                        (first_row_y + i * inter_box_y - offset) + box_size,
                        (first_row_x + j * inter_box_x):
                        (first_row_x + j * inter_box_x) + box_size]
                else:
                    img = self.frame[
                        (first_row_y + i * inter_box_y):
                        (first_row_y + i * inter_box_y) + box_size,
                        (first_row_x + j * inter_box_x):
                        (first_row_x + j * inter_box_x) + box_size]

                pkg_colour = get_qr_data(img)
                self.package_colour.append(pkg_colour)

                if pkg_colour == "red":
                    dictionary = {"id": "Inventory", "Team Id": "VB#1202", "Unique Id": "isAmiTvb",
                                  "SKU": "%s" % ("R" + str(i) + str(j)) + datetime.now().strftime(
                                      '%m') + datetime.now().strftime('%y'),
                                  "Item": "Medicines", "Priority": "HP",
                                  "Storage Number": "R{} C{}".format(str(i), str(j)),
                                  "Cost": "450", "Quantity": "1"}
                    ACTION_CLIENT.send_goal_pls(dictionary)
                    time.sleep(0.2)
                    self.counter += 1

                elif pkg_colour == "yellow":
                    dictionary = {"id": "Inventory", "Team Id": "VB#1202", "Unique Id": "isAmiTvb",
                                  "SKU": "%s" % ("Y" + str(i) + str(j)) +
                                         datetime.now().strftime('%m') +
                                         datetime.now().strftime('%y'),
                                  "Item": "Food", "Priority": "MP",
                                  "Storage Number": "R{} C{}".format(str(i), str(j)),
                                  "Cost": "250", "Quantity": "1"}
                    ACTION_CLIENT.send_goal_pls(dictionary)
                    time.sleep(0.2)
                    self.counter += +1
                elif pkg_colour == "green":
                    dictionary = {"id": "Inventory", "Team Id": "VB#1202", "Unique Id": "isAmiTvb",
                                  "SKU": "%s" % ("G" + str(i) + str(j)) + datetime.now().strftime(
                                      '%m') + datetime.now().strftime('%y'),
                                  "Item": "Clothes", "Priority": "LP",
                                  "Storage Number": "R{} C{}".format(str(i), str(j)),
                                  "Cost": "150", "Quantity": "1"}
                    ACTION_CLIENT.send_goal_pls(dictionary)
                    time.sleep(0.2)
                    self.counter += 1

                pkg_colour_list = ','.join(self.package_colour)
                PUB.publish(pkg_colour_list)


def listener():
    """
    The main subscriber actions.
    """

    ob_1 = Camera1()

    while not rospy.is_shutdown():
        ob_1.calculate()
        RATE.sleep()


if __name__ == '__main__':
    try:
        rospy.sleep(7)
        rospy.init_node('node_package_detect', anonymous=True, disable_signals=True)
        PUB = rospy.Publisher('/package_colour', String, queue_size=10)
        RATE = rospy.Rate(50)

        ACTION_CLIENT = RosIotBridgeActionClient()

        listener()
    except rospy.ROSInterruptException:
        pass
