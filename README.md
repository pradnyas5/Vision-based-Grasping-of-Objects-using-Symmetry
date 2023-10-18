# Vision-based-Grasping-of-Objects-using-Symmetry

    1.	Introduction/Abstract
The "Grasping Assuming Symmetry" project leverages the nature of symmetry present in many objects. Using a depth camera, our approach captures real-time 3D representations of the environment, focusing on object symmetries to optimize grasping points. This integration of depth-sensing technology and algorithmic innovation offers a promising advancement in robotic manipulation, ensuring more stable and efficient interactions with a multitude of objects in real-world scenarios.

    2. Implementation
Step 1: Downsampling and major plane removal
●	The point cloud data is first downsampled using a voxel grid to reduce the number of points, making subsequent processes computationally efficient.
●	Plane segmentation using RANSAC is applied to remove the plane from the point cloud. This ensures that only the object of interest remains in the cloud.
