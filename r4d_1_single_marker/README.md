Steps:

1. Build and setup catkin environment.

```
cd ../.. && catkin_make && source devel/setup.bash && cd -
```

2. Detect markers in `dataset_table.bag`, write detections to `markers_table.bag`.

```
python scripts/process_rosbag.py
```

3. Run camera pose estimation (OpenCV and GTSAM).

```
roslaunch r4d_1_single_marker eval.launch
```

4. Generate plot of z-position (publish to plot.ly).

```
python scripts/plot.py
```
