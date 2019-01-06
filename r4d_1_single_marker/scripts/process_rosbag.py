import cv2
import rosbag
from rcars_detector_msgs.msg import Tag

def main():
    inputPath = "/media/nvidia/nvme256/rcarsdata/dataset_table.bag"
    outputPath = "/media/nvidia/nvme256/fiducial_slam/dataset_table_tagged.bag"
    allTopics = [ "/cam0/camera_info", "/cam0/image_raw", "/imu0", ]

    try:
        inputBag = rosbag.Bag(inputPath)
        outputBag = rosbag.Bag(outputPath, "w")

        for topic, msg, t in inputBag.read_messages(topics=allTopics):
            print(topic, t)
            if topic == "/tf" and msg.transforms:
                outputBag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outputBag.write(topic, msg, msg.header.stamp if msg._has_header else t)

    finally:
        inputBag.close()
        outputBag.close()

if __name__ == "__main__":
    main()
