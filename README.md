# Structure from motion exploration

With just two frames

<img src="testdata/reconstruction/kleenex/two_frames/frame_1.jpg" width="300" height="300"/>
<img src="testdata/reconstruction/kleenex/two_frames/frame_35.jpg" width="300" height="300"/>

feature matching
![visualization](images/matches_visualization.png)

point cloud reconstruction is

<img src="images/kleenex_2.png" width="400" height="200"/>

With 13 frames it is

<img src="images/kleenex_13.png" width="400" height="200"/>




## This comes with

* Chopping frames from video file
* Camera calibration
* Reconstruction (from frame directory to point cloud PLY)