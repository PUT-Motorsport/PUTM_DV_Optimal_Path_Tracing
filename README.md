# PUTM_DV_Optimal_Path_Tracing

The goal of this set of algorithms is to decide the optimal path basing on the two sets of track bounding cones.

**Input:**
2D cartesian coordinates of two sets of track bounding cones, yellow and blue ones

**Output:**
An optimal path spline (*exact representation tbd*).

## Dependencies

- eigen
- boost

```
sudo apt install libeigen3-dev libboost-all-dev libompl-dev
```

## RRT Alogirthm

RRT implementation [repo](https://github.com/RoboJackets/rrt)


### Raceline

After restoring track bounds, the raceline will be extrapolated from the data. 

The raceline is found by taking a set of same-side points and searching the other side for the closest point.
Some optimisations are possible, e.x. narrowing the search region by heuristics or discarding some points and restoring the track by spline interpolation.

The centerline will be represented as a spline of points at a set interval. The data point vector will contain:

$$
d = \begin{bmatrix} p_{centerline} \space w_l \space w_r\end{bmatrix}
$$

Where the $w_l$ and $w_r$ denote the maximum distance the point may be altered without crossing the bounds.

### Minimal curvature

The track curvature can be calculated by:

$$
\kappa = \frac{||d\overrightarrow{T}||}{||d\overrightarrow{s}||}
$$

The curvature of the raceline will be minimized iteratively until a satisfactory result is obtained (condition tbd).
