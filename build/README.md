# Optimise

We currently run on small toy meshes and processing is slow.
It's probably O(n^2) in places
This will make it unreasonable to run on large point clouds.
We should take this opportunity to optimise the code.

We'll do it in a structured way, obtaining performance stats for loading and smoothing and using e.g. valgrind to find bottlenecks.

# Next
Implement a timing test; simple test case that loads horse in FieldOptimiser.cpp



Profiler finds that most time is spent in `compute_tangents_for_tier_and_frame` with the bulk of the time split between matrix multiplication  and reprojection.



# Results

## Debug mode
+------------------+--------+----------+-------+-------+-------------------------------------+
| Test             | Frames | Vertices | Edges | Faces | Times (µs)                          |
+------------------+--------+----------+-------+-------+-------------------------------------+
| Sphere_20x20     |      1 |      382 |           400 | 672 384 445 517 194 300             |
+------------------+--------+----------+-------+-------+-------------------------------------+
| Sphere_32x32     |      1 |                          | 1294 1370 840 875 871 728           |
+------------------+--------+----------+-------+-------+-------------------------------------+
| cloth2           |      4 |                          | 15450, 26010, 29621, 15015, 29313   |
+------------------+--------+----------+-------+-------+-------------------------------------+

## Release mode
+------------------+--------+----------+-------+-------+----------------------------------------+
| Test             | Frames | Vertices | Edges | Faces | Times (µs)                             |
+------------------+--------+----------+-------+-------+-----------+---------------+------------+
|                  |        |          |       |       | Load (ms) | Optimise (ms) | Total (ms) |
+------------------+--------+----------+-------+-------+-----------+---------------+------------+
| Sphere_20x20     |      1 |      382 |           400 | 59 39 18 45                            |
+------------------+--------+----------+-------+-------+----------------------------------------+
| Sphere_32x32     |      1 |                          | 114 120 73 77 76                       |
+------------------+--------+----------+-------+-------+----------------------------------------+
| cloth2           |      4 |                          | 758 746 912                            |
|                  |      3 |                          | 638 622 635                            |
|                  |      2 |                          | 296 263 225                            |
+------------------+--------+----------+-------+-------+----------------------------------------+
|                  |      1 |   8431   |       | 16843 |     73    |    52         |    125     |
+------------------+--------+----------+-------+-------+----------------------------------------+
| mini-horse       |      3 |  25293   |       | 50529 | 956117                                 |
|                  |      1 |   8431   |       | 16843 | 68,956    | 1,875         | 70,831     |
+------------------+--------+----------+-------+-------+----------------------------------------+

At higher volumes of points, tier matching still pretty constant @ 22ms on tier 0 while frames are 890µs for 3 frames
suggesting performance of the frame mathing could be improved; not O(n)?

# Standalone Test Case
Loading horse-04 (release code) 69,079ms, 70.198ms,  i.e. 70 seconds!

Most time is spent :
35.94 s   51.3%	1.18 s	 	                animesh::Path::Path(animesh::Path const&) (new)
14.98 s   21.4%	14.81 s	 	                free_tiny


## Getting tangents once
[----------] 1 test from TestPerformance
[ RUN      ] TestPerformance.timeLoadHorse
              Load time (ms): 72524
              compute tangents (7,0) (ms): 9
[       OK ] TestPerformance.timeLoadHorse (72565 ms)
[----------] 1 test from TestPerformance (72565 ms total)




# First optimisation
We can either reduce the number of calls to or the performance of `compute_tangents_for_tier_and_frame`

1. We can cache tangents at each tier and frame and only recompute them as needed
  1.1 Store tangents first time they are needed
  1.2 Set a flag indicating that they are good for the given tier and frame
  1.3 (Important) invalidate cache for tier and frame as needed
    1.3.1 When smooth once is called
    1.3.2 When randomise is called
    1.3.3 When a new object is loaded

2. We can speed up computation of the tangents.
  2.1


Sphere_20x20 seems to have tiers and frames section calculations being equally expensive and both seemingly scaling linearly with the number of nodes.
To optimise frames we might copy all data into a matrix, multiply out and copy back.
This will force the use of Eigen libraries which are better optimised than my handrolled code.
OTOH we should alos run timing tests with optimisation on...
compute_tangents_for_tier_and_frame (5,0) from tier : 0
	tiers  : 2506008ns
	frames : 1779ns
	total  : 2507787ns
compute_tangents_for_tier_and_frame (5,0) from tier : 5
	tiers  : 1289ns
	frames : 1121ns
	total  : 2410ns
compute_tangents_for_tier_and_frame (4,0) from tier : 4
	tiers  : 1161ns
	frames : 1094ns
	total  : 2255ns
compute_tangents_for_tier_and_frame (3,0) from tier : 3
	tiers  : 2393ns
	frames : 2203ns
	total  : 4596ns
compute_tangents_for_tier_and_frame (2,0) from tier : 2
	tiers  : 3247ns
	frames : 2834ns
	total  : 6081ns
compute_tangents_for_tier_and_frame (1,0) from tier : 1
	tiers  : 4625ns
	frames : 4740ns
	total  : 9365ns
compute_tangents_for_tier_and_frame (0,0) from tier : 0
	tiers  : 9257ns
	frames : 8801ns
	total  : 18058ns
