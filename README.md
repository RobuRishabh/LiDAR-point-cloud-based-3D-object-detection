# LiDAR point-cloud based 3D object detection
Object detection is a key component in advanced driver assistance systems (ADAS), which allow cars to detect driving lanes and pedestrians to improve road safety. This report describes a modern approach for 3D Object Detection using LiDAR while driving on the road. The system includes a Velodyne VLP-16 LiDAR sensor to capture real-time scenarios. Using Open3d, we perform the following: segmentation, RANSAC, DBSCAN, Voxel-Grid Downsampling, clustering, and detection using bounding boxes. The experimental results confirm the detection of objects such as pedestrians, cyclists, and other vehicles.

To get the LiDAR_pedestrian_bicyclist_cars dataset ROSbag file visit this link:
- https://northeastern-my.sharepoint.com/:f:/r/personal/singh_risha_northeastern_edu/Documents/LiDAR%20bicycle_Pedestrian%20data?csf=1&web=1&e=rjat17

# Method
LiDAR is an acronym for Light Detection And Ranging. Eye-safe laser beams are used to create a 3D representation of the environment. The LiDAR sensor emits pulsed light waves from a laser into the surrounding area. These pulses bounce off surrounding objects and return to the sensor. The sensor uses the time it took for each pulse to return to the sensor to calculate the distance it traveled. Repeating this process multiple times per second creates a real-time 3D point cloud map of the environment.

The dataset are available here. The dataset was collected using Velodyne’s VLP-16 LiDAR model. Each packet contains the data from 24 firing sequences in 12 data blocks where each data block contains information from two firing sequences of 16 lasers.

# Principle behind Algorithms
# 1. Voxel-Grid Down-sampling
After extraction, we obtain a large number of points to process which we wish to reduce for computational efficiency. However, we also wish to retain the data’s key features. Because of this, we use Voxel grids which are 3D boxes or cells that can hold multiple points. The points present in each voxel (i.e., 3D box), are approximated with their centroid. This represents an accurate structure of the surface by points that are equidistant from each other. While retaining the essential features of the data set, we reduced the number of points seen in each frame to around 6,500 from the original 23,000 data points.
# 2. RANSAC: RANdom Sampling and Consensus
The core RANSAC algorithm is fairly straightforward:
1. Select a random set of points (3 points to form a plane).
2. Calculate the parameters required for the plane equation.
3. Calculate the distance of all the points in the point cloud from the plane.
4. If the distance is within the THRESHOLD then add the point as an inlier.
5. Store the plane points and points having the maximum number of inliers.
6. Repeat the process again for MAX-ITERATIONS.

<img src="Pictures\RANSAC1.png" width=700px>
<img src="Pictures\RANSAC2.png" width=700px>
<img src="Pictures\RANSAC3.png" width=700px>
After RANSAC is complete, the plane having the maximum number of inliers is the best estimate of the ground plane (i.e road plane). Using this model-based algorithm, the ground plane can be eliminated, hence the obstacles in the field of view of the sensor can be more efficiently localized and detected.

# 3. DBSCAN: Density-Based Spatial Clustering of Applications with Noise
DBSCAN stands for Density-Based Spatial Clustering of Applications with Noise. The algorithm can
be summarized as follows:
1. Pick a point in the dataset (until all points have been visited).
2. If there are at least MINPOINTS points within a radius of EPS to the point selected, then we consider all these points to be part of the same cluster.
3. The clusters are then expanded by repeating this calculation for each neighboring point. DBSCAN is a robust mechanism for clustering by removing noise as outliers. Its advantages over other methods include being able to detect arbitrary clusters and being independent of the requirement to predefine the number of clusters like k-means clustering.

# 4. Bounding box Detection
To draw bounding boxes around the clusters of points, we get the labels of each cluster and group them together. Using these labels, we get the indices of the actual points to get the axis-aligned bounding boxes.

# Analysis of approach
To begin our approach, we can first visualize the data in point clouds. By extracting our data with Velodyne-decoder and using Open3d for point cloud processing, we can see the resulting point clouds of a single frame in our data. To segment the road plane out of the data, we use the previously
described RANSAC algorithm. The road plane, labeled in red, is well-defined and can be separated from the data to reduce the complexity and possible noise.

  Initial point clouds of a single frame      |      Result after segmentation        |
:-------------------------:|:------------------------:|
| <img src="Pictures\Downsampling.jpg" width=500px> | <img src="Pictures\RANSAC.jpg" width=500px> |

With only the outlier points, we still have around 23,000 points in a frame, therefore, using Voxel downsampling helps us reduce this complexity to around 6,500 points. We can see a before and after. Despite reducing the complexity by a factor of 3, the contents seen in the point clouds are not affected as clusters of points are not affected, rather the number of points that form those clusters.

   Before running voxel downsampling      |       After running voxel downsampling        |
:-------------------------:|:------------------------:|
| <img src="Pictures\downsampling2.jpg" width=500px> | <img src="Pictures\Downsampling1.jpg" width=500px> |

Finally, using DBSCAN, we can cluster the points into objects. We can see the final objects detected. Given these clusters, we can place bounding boxes on the detected objects.

# Final Results
we were able to detect the objects accurately in real time. As seen from the final results, it may not always be perfectly accurate because of the down-sampling we did with voxels, which may result in data loss due to saturation of point density. <- Not entirely accurate But the current results seen show that our task of detection can be achieved with satisfying results.

   Results after clustering      |       Clusters with bounding boxes        |
:-------------------------:|:------------------------:|
| <img src="Pictures\Clustering.jpg" width=500px> | <img src="Pictures\Bounding Box.jpg" width=500px> |

# References :
1. Velodyne lidar puck. https://www.amtechs.co.jp/product/VLP-16-Puck.pdf.
2. Downsampling a pointcloud using a voxelgrid filter. https://adioshun.gitbooks.io/pcl/content/Tutorial/Filtering/pcl-cpp-downsampling-a-pointcloud-using-a-voxelgrid-filter.html, 2022.
3. Downsampling a pointcloud using a voxelgrid filter — point cloud library 0.0 documentation. https://pcl.readthedocs.io/en/latest/voxel_grid.html, 2022.
4. What is lidar? learn how lidar works, velodyne lidar. https://velodynelidar.com/what-is-lidar/, 2022.
5. Nagesh Chauhan. Dbscan clustering algorithm in machine learning - kdnuggets. https://www.kdnuggets.com/2020/04/dbscan-clustering-algorithm-machine-learning.html, 2022.
6. Martin Simon, Stefan Milz, Karl Amende, and Horst-Michael Gross.Complex-yolo: Real-time 3d object detection on point clouds. arXiv, 2018.
7. Martin Valgur. Velodyne decoder. https://github.com/valgur/velodyne_decoder, 2022.
8. Leah A. Wasser. The basics of lidar - light detection and ranging - remote sensing. https://www.neonscience.org/resources/learning-hub/tutorials/lidar-basics, 2022.
9. Qian-Yi Zhou, Jaesik Park, and Vladlen Koltun. Open3D: A modern library for 3D data processing. arXiv:1801.09847, 2018.
