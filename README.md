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

This is a very basic rudimentary SfM. For the same 13 input images [Colmap](https://github.com/colmap/colmap)
reconstructs both sparse and dense in 2-3 minutes, which is incredible in quality and speed:

## COLMAP classic
### Sparse 
<img src="images/sparse.jpg"/>

### Dense (Poisson)
<img src="images/frame000000.png" />

## Volumetric rendering

I ran the very same images and camera poses through NeRF and Gaussian Spatting. 

### NeRF
<video src="images/nerf-tiny.mp4" width="400" height="200" controls>  
  Your browser does not support the video tag.
</video>

### Gaussian Spatting
<video src="images/traj_tiny.mp4" width="400" height="200" controls>  
  Your browser does not support the video tag.
</video>

