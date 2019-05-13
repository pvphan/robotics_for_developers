terminal interface to generate the above data from sections of rosbags
in debug mode, store full state of system at each time step

when browsing debug data, can step through time steps to observe:
    - map in 3D with pose history (general feel for what the system is doing). visual for covariance of each pose would be really cool
    - high level performance statistics: which time stamp, current certainty of pose, num nodes / edges in factor graph, computation time required for this step
    - rationale behind new step:
        - which data was selected (current and previous key frame)
        - 2D point correspondences (data association)
        - change to factor graph (effect on backend)

eventual online: sensors -> python node
initial offline: sensors -> [rosbag] -> "player" layer -> python node

sensor node: can poll sensor (grab frame, get IMU data) or read ros message
    as long as handler methods are the same between both input methods
        - (record) image.array -> np.array --> ros image message --> rosbag.write
        - (playback) ros image message -> cv_bridge -> np.array
        - (online) image.array -> np.array

map out SFM_example and r4d_3_imu on whiteboard. consolidate the good parts, derive custom example using rcars dataset
