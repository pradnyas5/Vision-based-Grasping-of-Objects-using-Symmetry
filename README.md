# Vision-based-Grasping-of-Objects-using-Symmetry

## 1. Introduction/Abstract
The "Grasping Assuming Symmetry" project leverages the nature of symmetry in many objects. Our approach uses a depth camera to capture real-time 3D representations of the environment, focusing on object symmetries to optimize grasping points. This integration of depth-sensing technology and algorithmic innovation offers a promising advancement in robotic manipulation, ensuring more stable and efficient interactions with many objects in real-world scenarios.

## 2. Implementation   

### Step 1: Downsampling and major plane removal
●	The point cloud data is first downsampled using a voxel grid to reduce the number of points, making subsequent processes computationally efficient.    
●	Plane segmentation using RANSAC is applied to remove the plane from the point cloud. This ensures that only the object of interest remains in the cloud.

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/549f6d25-ad91-4470-922c-a175b4d7ca08)    
*Input Point Cloud*

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/6840db1d-8f52-4443-b7ab-3f7b9a2a3406)     
*Output Point Cloud*    

### Step 2: Point Cloud Completion
●	A symmetry-based approach is utilized to complete the point cloud. The best symmetry plane is determined based on visibility constraints.    

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/9af92a5a-7be6-4e9d-bd3c-2454c7aa7fc2)   
*Incomplete and Complete Point Cloud*

●	We first create all possible planes oriented at different angles.    
●	We then mirror every point in the current point cloud about each of these planes.      
●	Once the mirrored point is obtained, we then evaluate the plane in consideration based on the condition: (distance - (distance*distance)/dist_threshold)*4/dist_threshold.    
●	The above condition is nothing but the visibility score metric which is calculated for all planes.   
●	Finally, we select the best plane that has the highest score to generate the complete point cloud.   

### Step 3: Normal Estimation   
●	Normals are calculated using the pcl::NormalEstimation class. This method estimates normals based on the local neighborhood of each point.   
●	KDTree is used for efficient spatial neighbor searching.   
●	The computed normal at a point is influenced by the positions of its neighbors. The method calculates the best-fitting plane for the neighbors and determines the plane's perpendicular as the normal.    
●	Normals are then visualized as small arrows emanating from points in the point cloud, pointing in the direction of the normal as seen in the figure below.   

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/4c9fafd8-398e-4101-ace2-36f76551fac1)    
*Surface Normals*   

### Step 4: Grasp Detection   
We have calculated the best grasp contacts for the robot when interacting with 3D point cloud data completed in the previous steps. The key steps of this process include:
1.	Input parameters: 
This includes the completed point cloud represented using the Point Cloud Library (PCL) data structure and another point cloud containing the surface normals corresponding to the points in the completed point cloud.   
2.	Nearest Point Search:
The nearest point to the centroid of the completed point cloud is calculated using KDTree and the result is stored. This is done by calculating the squared distance metric of each point in the completed point cloud from the centroid (mean) of the completed point cloud.   

3.	Angle calculation:
The function then calculates the angles between the normals of each point in the point cloud and the vectors from the nearest point (contact1) to these points. It uses the dot product and vector magnitudes to calculate the angle between the normal and the vector. The function iterates through all points in complete_points and updates the current point (contact2) based on the minimum angle. The goal is to find a point that, when grasped along with contact1, results in the smallest angle between the contact normals and the grasp direction. This is based on the principle shown below:

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/1894d9c2-9b58-4c35-b01b-6e9a623aab20)

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/d75aabe7-13cd-4632-a99f-6ec9eaa13896)   

Among these, the first grasp is the most stable one as the angle between the normals at the contact points and the vector between the contact points is minimal.   

4.	Best grasp:
Thus, we obtain the second point for grasping which will be the most stable grasping point along with the point nearest to the centroid of the completed point cloud.

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/34a8c508-491a-4400-b419-f54aca0f4fd7)

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/a54782d6-0df6-43c0-ba6e-dc56f4aade8b)

*Best Grasping Contacts (in red)*

## 3. Results of Point Cloud Completion

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/7bd6e168-a989-425b-aff6-49eb649bb128)    
*Coke Can*

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/9e14a30e-6505-4918-8196-0b557af8db20)   
*Beer Can*

![image](https://github.com/pradnyas5/Vision-based-Grasping-of-Objects-using-Symmetry/assets/93536494/ebe37122-bd1d-4021-b12a-4be79d2bd1e4)   
*Cricket Ball*









