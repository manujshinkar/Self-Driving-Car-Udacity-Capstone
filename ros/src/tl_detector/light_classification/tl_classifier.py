from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #implement light color prediction
        light = TrafficLight.UNKNOWN
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_threshold = np.array([0,50,50])
        upper_threshold = np.array([10,255,255])
        thresholded_image1 = cv2.inRange(image, lower_threshold , upper_threshold)

        lower_threshold = np.array([170,50,50])
        upper_threshold = np.array([180,255,255])
        thresholded_image2 = cv2.inRange(image, lower_threshold , upper_threshold)

        weighted_image = cv2.addWeighted(thresholded_image1, 1.0, thresholded_image2, 1.0, 0.0)
        blurred_image = cv2.GaussianBlur(weighted_image,(15,15),0)

        circles = cv2.HoughCircles(blurred_image,cv2.cv.CV_HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)

        if circles is not None:
            light = TrafficLight.RED

        return light
