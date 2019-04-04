Usage:
=================

1. download KITTI odometry ground truth datas to "PATH/TO/WORKSPACE/dataset/poses"
2. build target :
```
g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp
```
3. put target <evaluaate_odometry> to your SLAM eval workspace.
4. usage:
```
./evaluate_odometry {PATH/TO/YOUR/SLAM/RESULT} {KITTI_SEQUENCE_NUMBER}

for example:
./evaluate_odometry ./CameraTrajectory.txt 2
```

## File format
### error format
first_frame r_err(rad/m) t_err(%) length(m) speed(m/s)

### path format
gt.x gt.y re.x re.y

### error plot format
tl: length(m) ave_t_err(%)
rl: length(m) ave_r_err(rad/m)
ts: speed(m/s) ave_t_err(%)
rs: speed(m/s) ave_r_err(rad/m)

### stats
tatal_avg_t_err(%) tatal_avg_r_err(rad/m)


