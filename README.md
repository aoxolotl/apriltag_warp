## Image rectification using AprilTag
[AprilTags](https://april.eecs.umich.edu/software/apriltag) are fiducial
markers which provide the position and orientation in 3D space relative to a
camera. AprilTags are commonly used in robotics for localization and
navigation.   

Here, AprilTags are being used to rectify images. A skewed image
taken with a normal camera can be made parallel to the view frame of the camera
with the help of AprilTags. The homography of a planar surface, for example, a
wall, can be calculated by detecting an AprilTag on the plane of the surface
given the dimensions of the tag and the focal length of the camera. The
homography matrix obtained from the detection is then used to warp the images
and make them parallel to the wall. If the size of the AprilTag is known in
real-world coordinates, the length between any two points in the image can be
calculated after warping.

### Build
Depends on `cmake` and `opencv` for building.
```
$ mkdir build
$ cmake ..
$ make -j
```

### Usage
```
$ apriltag_warp <filename.png> 
```
Writes out the rectified image.
