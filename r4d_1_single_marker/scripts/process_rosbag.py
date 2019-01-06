import pdb
from pprint import pprint

import apriltag
import cv2
import numpy as np
import rosbag

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from rcars_detector_msgs.msg import Tag, TagArray

def quaternion_from_matrix(matrix, isprecise=False):
    """
    From: https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
    Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.
    """
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                      [m01+m10,     m11-m00-m22, 0.0,         0.0],
                      [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                      [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

def getRosPoseFromMatrix(pose4x4):
    quaternion = quaternion_from_matrix(pose4x4[:3,:3])
    rosQuaternion = Quaternion()
    rosQuaternion.w = quaternion[0]
    rosQuaternion.x = quaternion[1]
    rosQuaternion.y = quaternion[2]
    rosQuaternion.z = quaternion[3]
    pose = Pose(Point(*pose4x4[:3,3]), rosQuaternion)
    return pose

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
                    debugImage = cv2.cvtColor(imageUndistorted, cv2.COLOR_GRAY2BGR)

                    # detect april tag, write to output bag in /rcars/detector/tags topic
                    tagArray = TagArray()
                    detections = detector.detect(imageUndistorted)

                    for detection in detections:
                        cameraParams = K[0,0], K[1,1], K[0,2], K[1,2]
                        M, _, _ = detector.detection_pose(detection, cameraParams)

                        tag = Tag()
                        tag.id = detection.tag_id
                        rosCorners = map(lambda x: Point(x[0], x[1], 0.), detection.corners)
                        tag.corners = rosCorners
                        tag.pose = getRosPoseFromMatrix(M)

                        tagArray.tags.append(tag)

                        # draw on debug image
                        for corner in detection.corners:
                            cornerInt = map(lambda x: int(round(x)), corner)
                            cv2.circle(debugImage, tuple(cornerInt), 5, (0, 255, 0))

                    if detections:
                        cv2.imshow("debugImage", debugImage)
                        cv2.waitKey()

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
