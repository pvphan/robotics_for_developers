from pprint import pprint

import apriltag
import cv2
import numpy as np

np.set_printoptions(precision=4)

def main():
    tagSize = 0.16
    detector = apriltag.Detector()
    imageUndistorted = cv2.imread("imageUndistorted.png", 0)
    cameraParams = (466.19345, 464.67789, 368.5271, 228.25047)
    detections = detector.detect(imageUndistorted)
    for detection in detections:
        cMm, _, _ = detector.detection_pose(detection, cameraParams,
                                            tag_size=tagSize)
        pprint(cMm)

if __name__ == "__main__":
    main()
