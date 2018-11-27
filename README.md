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

1. Group all holes(cylinders) by axis direction
2. **Under the assumption that all holes are composed of two half cylinders surface**, filter out cylinders which have no pairs and is single. i.e. round angle
3. Given angle tolerance, find axis direction pairs between two parts that are to be aligned.
4. Rotate one of the parts to parallelize them according to the holes axis pair that has the closest angle
5. Project all center of the holes with given direction to the contact planes. Connect all points and compute all the relative distance and angles between each two connection. By doing so, we can get a relationship table
6. Compare the relationship table from two parts and eliminate the holes(point) which doesn't exist in both parts.
7. The rest of points could be the holes which are able to be aligned. **(Holes' diameter is not considered in my algorithm till now)**
8. If there is only one hole the matching process is different from matching two holes.
9. If there are more than two holes, it uses SVD to match them.