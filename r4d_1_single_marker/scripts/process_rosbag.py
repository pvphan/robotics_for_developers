import pdb
from pprint import pprint

import apriltag
import cv2
import numpy as np
import rosbag

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
from rcars_detector_msgs.msg import Tag, TagArray

def main():
    cvBridge = CvBridge()
    detector = apriltag.Detector()

    inputPath = "/media/nvidia/nvme256/rcarsdata/dataset_table.bag"
    outputPath = "/media/nvidia/nvme256/fiducial_slam/dataset_table_tagged.bag"
    allTopics = [ "/cam0/camera_info", "/cam0/image_raw", "/imu0", "/vicon/auk/auk" ]
    tagsTopic = "/rcars/detector/tags"
    D = None
    K = None

    try:
        inputBag = rosbag.Bag(inputPath)
        outputBag = rosbag.Bag(outputPath, "w")

        for topic, msg, t in inputBag.read_messages(topics=allTopics):
            if topic == "/tf" and msg.transforms:
                outputBag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                if topic == "/cam0/image_raw":
                    # can't detect tag pose if we don't have camera parameters
                    if D is None or K is None:
                        continue

                    print(topic, t)
                    imageDistorted = cvBridge.imgmsg_to_cv2(msg, "mono8")

                    # undistort image
                    h, w = imageDistorted.shape[:2]
                    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(K, D, (w,h), 0, (w,h))
                    imageUndistorted = cv2.undistort(imageDistorted, K, D, None, newCameraMatrix)

                    # detect april tag, write to output bag in /rcars/detector/tags topic
                    tagArray = TagArray()
                    detections = detector.detect(imageUndistorted)

                    #cv2.imshow("image", imageUndistorted)
                    #cv2.waitKey()

                    for detection in detections:
                        cameraParams = K[0,0], K[1,1], K[0,2], K[1,2]
                        M, _, _ = detector.detection_pose(detection, cameraParams)

                        tag = Tag()
                        tag.header = msg.header
                        tag.id = detections.tag_id

                        pdb.set_trace()
                        for corner in detection.corners:
                            tag.corners.append(Point(corner))

                elif topic == "/cam0/camera_info":
                    K = np.array(list(msg.K)).reshape((3,3)).astype(np.float32)
                    D = np.array(list(msg.D)).astype(np.float32)
                else:
                    outputBag.write(topic, msg, msg.header.stamp if msg._has_header else t)

    finally:
        inputBag.close()
        outputBag.close()

if __name__ == "__main__":
    main()
