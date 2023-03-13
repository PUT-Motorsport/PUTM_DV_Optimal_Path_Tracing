# PUTM_DV_Optimal_Path_Tracing

The repo has two packages:
- package_opt, which takes a set of cones as an input and outputs a (somewhat optimized) path
- package_control, which is responsible for steering the car. It depends on package_opt as its input

## Package opt

The first stage is simple filtering of input cones (for now, discarding those behind the car). 
Then, cones that are in a set range are selected for sampling. Their center is the mean and the difference between the farthest cone and the center is the standard deviation (in two dimensions).
Coordinates are sampled from a multivariate Gaussian distribution (*though a different distribution may give better results?*).
The points on the path are then moved to the middlepoints of delaunay triangles vertices with which they intersect.

## Package control

Package control uses Pure Pursuit to follow package_opt's path.
