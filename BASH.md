## KAIST
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml ~/SLAM_WORK/DataSets/rgbd_dataset_freiburg1_desk
./Examples/KAIST/mono Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_left.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban22-highway

./Examples/KAIST/stereo Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban22-highway

./Examples/KAIST/stereo Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban30-gangnam

./Examples/KAIST/stereo Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_38.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban38-pankyo

./Examples/KAIST/stereo Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_38.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban39-pankyo

### wheel
./Examples/KAIST/stereo_wheel Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_wheel_39.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban39-pankyo

./Examples/KAIST/stereo_wheel Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_wheel_26.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban30-gangnam

./Examples/KAIST/stereo_wheel Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_wheel_26.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban26-dongtan
./Examples/KAIST/stereo Vocabulary/ORBvoc.txt Examples/KAIST/KAIST_stereo_26.yaml /media/liu/KINGSTON/Complex_Urban_LiDAR_Data_Set/urban26-dongtan


### evo
evo_traj tum --ref=/home/liu/SLAM_WORK/evo/test/global_poseTUM.txt /home/liu/SLAM_WORK/evo/test/KeyframeTrajectoryTUMchange.txt -a  -p

evo_ape tum /home/liu/SLAM_WORK/evo/test/global_poseTUM.txt /home/liu/SLAM_WORK/evo/test/KeyframeTrajectoryTUMchange.txt -r full -va --plot --plot_mode xy
evo_rpe tum /home/liu/SLAM_WORK/evo/test/global_poseTUM.txt /home/liu/SLAM_WORK/evo/test/KeyframeTrajectoryTUMchange.txt -r full -va --plot --plot_mode xy

## kitti
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml /media/liu/KINGSTON/Kitti/data_odometry_gray/dataset/00
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml /media/liu/KINGSTON/Kitti/data_odometry_gray/dataset/01

### evo
evo_traj kitti --ref=/media/liu/KINGSTON/Kitti/dataset/poses/01.txt /home/liu/SLAM_WORK/ORB_SLAM2/result/2024-01-14_15:17:14/TrajectoryKITTI.txt -a  -p
evo_ape kitti /media/liu/KINGSTON/Kitti/dataset/poses/01.txt /home/liu/SLAM_WORK/ORB_SLAM2/result/2024-01-14_15:17:14/TrajectoryKITTI.txt -r full -va --plot --plot_mode xz  
evo_rpe kitti /media/liu/KINGSTON/Kitti/dataset/poses/01.txt /home/liu/SLAM_WORK/ORB_SLAM2/result/2024-01-14_15:17:14/TrajectoryKITTI.txt  -r full -va --plot --plot_mode xz 









