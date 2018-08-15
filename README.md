# Relative 3D Pose Extraction
- Purpose  
  Analyze the geometric info from STEP models, and align them in order to improve the localization accuracy of camera



## Requirement
- pythonocc 1.8.1
- pyMesh x.x

## Algorithm description
### Plane alignment
1. Find all planes
2. Group them by the normal of plane  
   Parallel planes with reversed normal is considered as the same group
3. Compute the angle between each two normal and find the smallest pair
4. Find rotation axis for aligning the orientation of two planes:  
   - Direction of Axis: the cross vector from two normal vectors
   - Location of Axis: center of mass of the model
5. Rotate the model
6. Compute and find smallest plane distance  
   Due to small angle tolerance of two planes, the distance is computed by point to plane  
   How to select a point from one plane in a reasonable region is important.  
   ----------!!!!!!!----------  
   Now the algorithm takes point from location of plane which means it's a point from infinite plane. It doesn't guarantee the point is inside the boundary of surface  
   ----------!!!!!!!----------  
7. Move the model (Translation along normal)
### Hole alignment
