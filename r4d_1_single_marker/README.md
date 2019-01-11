Steps:

1. Download [`dataset_table.bag.zip`](http://www.adrlab.org/software/rcars/datasets/dataset_table.bag.zip) and unzip it to: `~/fiducial_slam/dataset_table.bag`

2. Build and setup catkin environment.

```
cd ../.. && catkin_make && source devel/setup.bash && cd -
```

3. Detect markers in `dataset_table.bag`, write detections to `markers_table.bag`.

```
python scripts/process_rosbag.py
```

4. Run camera pose estimation (OpenCV and GTSAM).

```
roslaunch r4d_1_single_marker eval.launch
```

5. Generate plot of z-position (publish to plot.ly).

```
python scripts/plot.py
```
