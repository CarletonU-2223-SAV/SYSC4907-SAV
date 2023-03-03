import cv2
import numpy as np
import rospy
from sensors.msg import SceneDepth
from sign_car_recognition.msg import DetectionResults, DetectionResult
from typing import List, Tuple
import math

DEPTH_RES = 256
MAX_DEPTH = 100
DIFF_WEIGHT = 0.20

# Bounding box parameters
GREEN = (0, 255, 0)
PADDING = 10
NORMAL_FONT = 0

# Object detection tuple indices
XMIN = 0
YMIN = 1
XMAX = 2
YMAX = 3
CONFIDENCE = 4
CLASS_NUM = 5
NAME = 6


class StopSignDetector:
    def __init__(self):
        self.stop_cascade = cv2.CascadeClassifier('stop_sign.xml')
        self.pub = rospy.Publisher("stop_sign_detection", DetectionResults, queue_size=1)
        rospy.init_node('new_stop_sign_detector', anonymous=True)

    def listen(self):
        rospy.Subscriber('airsim/scene_depth', SceneDepth, self.handle_image)
        rospy.spin()

    def handle_image(self, combine: SceneDepth):
        img1d = np.frombuffer(combine.scene.data, dtype=np.uint8)
        img_rgb = img1d.reshape(combine.scene.height, combine.scene.width, 3)
        res: List[DetectionResult] = self.detect_objects(img_rgb)

        # Match with scene depth
        depth = np.array(combine.depth_data, dtype=np.float32)
        depth = depth.reshape(combine.scene.height, combine.scene.width)
        depth = np.array(depth * (DEPTH_RES - 1), dtype=np.uint8)

        # Find median depth value for each detection box
        for detect in res:
            x1, x2 = math.floor(detect.xmin), math.floor(detect.xmax)
            y1, y2 = math.floor(detect.ymin), math.floor(detect.ymax)
            xdif, ydif = x2 - x1, y2 - y1
            x1_s, x2_s = math.floor(x1 + DIFF_WEIGHT * xdif), math.floor(x2 - DIFF_WEIGHT * xdif)
            y1_s, y2_s = math.floor(y1 + DIFF_WEIGHT * ydif), math.floor(y2 - DIFF_WEIGHT * ydif)

            sub = depth[y1_s:y2_s, x1_s:x2_s]
            med = np.median(sub)

            if math.isnan(med):
                rospy.loginfo(sub)
                rospy.loginfo(sub.shape)

            # Range of values is 0 to 100 m
            detect.depth = med / DEPTH_RES * MAX_DEPTH

            cv2.putText(img_rgb, f'{detect.name}: {detect.depth}', (x2 + PADDING, y2), NORMAL_FONT, 0.3, GREEN)

        cv2.imshow('Stop Signs', img_rgb)
        cv2.waitKey(1)

        rospy.loginfo(res)
        self.pub.publish(res)

    def detect_objects(self, img_rgb):
        # return detection results consisting of bounding boxes and classes
        gray_img = cv2.imdecode(img_rgb, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        stops = self.stop_cascade.detectMultiScale(gray, 1.3, 5)
        res_list: List[Tuple[float, float, float, float, float, int, str]]
        res_list = stops.pandas().xyxy[0].to_numpy().tolist()

        for (x, y, w, h) in stops:
            cv2.rectangle(img_rgb, (x, y), (x + w, y + h), GREEN, 2)

        detect_results = []
        # important detection classes we care about
        imp_classes = {
            11: 'stop sign'
        }
        for elem in res_list:
            # Skip adding the result if not a relevant class
            if elem[CLASS_NUM] not in imp_classes:
                continue

            dr = DetectionResult()
            dr.xmin = elem[XMIN]
            dr.ymin = elem[YMIN]
            dr.xmax = elem[XMAX]
            dr.ymax = elem[YMAX]
            dr.confidence = elem[CONFIDENCE]
            dr.class_num = elem[CLASS_NUM]
            dr.name = elem[NAME]
            detect_results.append(dr)

        return detect_results

if __name__ == "__main__":
    stop_sign_detect = StopSignDetector()
    stop_sign_detect.listen()
