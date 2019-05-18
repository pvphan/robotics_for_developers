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


platform TODOs
- calibrate camera
- figure out fixed inputs with very naive motion model
- calibrate IMU

tooling TODOs
- visualize robot state
    - pose history
    - landmarks in 3D
    - covariance (spheres?)
- sensor recording system
- sensor playback system
- [theory] data processing

theory
- stachniss lectures

roadmap (reverse order)
- online SLAM robot, simple goal [optimized]
- online stop-go SLAM robot, simple goal [planning]
- online stop-go SLAM robot, fixed movement pattern [iSAM2]
- offline stop-go SLAM robot, fixed movement pattern [actuation]
    - platform moves roughly to motion model
- offline stop-go SLAM robot, passive observation [perception]
    - sensors recording
    - playback system working


[theory] specify system conops
derisk: building, data capture, playback, online proc

build debug tools:
    - parse rosbag to snapshot folders
    - snapshot context visualizer: image folders, text files, factorgraph
prototype system on rcars dataset

gather data (per spec)
prototype system on custom dataset
