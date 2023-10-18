#include <iostream>
//#include <rclcpp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/conversions.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/extract_indices.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include "visualization_msgs/msg/marker_array.hpp"


using std::placeholders::_1;

bool sortcol(const std::vector<float>& v1, const std::vector<float>& v2){
	return v1[3]>v2[3];
}

class ObjectSegmentation : public rclcpp::Node
{
public:
  ObjectSegmentation() 
  : Node("object_segmentation")
  {
		subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/realsense/points",10,std::bind(&ObjectSegmentation::topic_callback, this, _1));
		complete_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/realsense/object_points",10);
		incomplete_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/realsense/input_object_points",10);
		contact_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/grasp_contacts",10);
		marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/point_normals",10);
		
		input_cloud = std::make_shared<pcl::PCLPointCloud2>();
		filtered_cloud = std::make_shared<pcl::PCLPointCloud2>();
		seg_cloud = std::make_shared<pcl::PCLPointCloud2>();
		incomplete_seg_cloud = std::make_shared<pcl::PCLPointCloud2>();
		contact_cloud = std::make_shared<pcl::PCLPointCloud2>();
		seg_point = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		complete_point = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
		//TODO uncomment this for viewer
		// viewer = std::make_shared<pcl::visualization::PCLVisualizer>("PCL Viewer");
		// viewer->setBackgroundColor (0.0, 0.0, 0.5);

		KDtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
		kd = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

		coefficients = std::make_shared<pcl::ModelCoefficients>();
		inliers = std::make_shared<pcl::PointIndices>();
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		transform_mat << 1.0000000,  0.0000000,  0.0000000,0, 0.0000000, -0.6955633,  0.7184648, -0.5 , 0.0000000, -0.7184648, -0.6955633, 1.0, 0.0, 0.0, 0.0, 1.0;
		Eigen::Matrix4f inv_matrix;
		inv_matrix << 1.0000000,  0.0000000,  0.0000000,0, 0.0000000, -0.6955633, -0.7184648,  0.37068316,  0.,  0.7184648,-0.6955633,  1.05479573, 0.0, 0.0, 0.0, 1.0;
		transform_inv_mat.matrix() = inv_matrix;

		callback_sec = 0.0;
		normals_valid = 0;

		_min_theta = -45*3.14/180;
		_max_theta = 45*3.14/180;
		x_steps = 6;
		y_steps = 6;
		theta_steps = 10;
		_visibility_score = -3;
		_min_dist = 0.01f;
	}


private:

    void topic_callback(const sensor_msgs::msg::PointCloud2& msg)
	{
		float time_in_sec = (float)(msg.header.stamp.sec%60)*1000.0 + (float)(msg.header.stamp.nanosec/1000000.0);
		if(callback_sec < time_in_sec && time_in_sec - callback_sec <= 2000.0) return;
		callback_sec = time_in_sec;
		sensor_msgs::msg::PointCloud2 msg_world;
		pcl_ros::transformPointCloud(transform_mat,msg,msg_world);

		//sensor_msg -> pcl::PointCloud2
		pcl_conversions::toPCL(msg_world,*input_cloud);
		RCLCPP_INFO_STREAM(this->get_logger(), "INPUT PCL size " << input_cloud->width * input_cloud->height);

		//Downsampling using VoxelGrid
		sor.setInputCloud(input_cloud);
		sor.setLeafSize (_min_dist,_min_dist,_min_dist);
		sor.filter(*filtered_cloud);
		RCLCPP_INFO_STREAM(this->get_logger(), "SAMPLED PCL size " << filtered_cloud->width * filtered_cloud->height);
		
		//Downsampled pcl::PointCloud2 -> pcl::PointXYZ
		pcl::fromPCLPointCloud2(*filtered_cloud,*seg_point);

		//Plane segmentation and removal
		int itr = 0;
		int initial_size = seg_point->size();
		while (seg_point->size () > 0.2 * initial_size && itr<6)
		{
			//Plane Fitting
			seg.setInputCloud (seg_point);
			seg.segment (*inliers, *coefficients);
			
			if (inliers->indices.size () == 0)
			{
			RCLCPP_ERROR_STREAM (this->get_logger(),"Could not estimate a planar model for the given dataset.\n");
			return;
			}

			// Extract the inliers
			extract.setInputCloud (seg_point);
			extract.setIndices (inliers);
			// Removing detected plane
			extract.setNegative (true);
			extract.filter (*seg_point);
			itr++;
		}

		computeFeatures(seg_point);
		_dist_threshold = (_max_y - _min_y);

		RCLCPP_INFO_STREAM(this->get_logger(), "Mean of PointCloud:" << _mean.x<<" "<< _mean.y<<" "<< _mean.z); 
		RCLCPP_INFO_STREAM(this->get_logger(), "Range of x:" << _min_x<<" "<< _max_x);
		RCLCPP_INFO_STREAM(this->get_logger(), "Range of y:" << _min_y<<" "<< _max_y);
		RCLCPP_INFO_STREAM(this->get_logger(), "Min image z:" << _min_im_z);
		RCLCPP_INFO_STREAM(this->get_logger(), "Visibility :" << _min_im_xz<<" <= x/z <= "<< _max_im_xz <<", "<< _min_im_yz <<" <= y/z <= "<< _max_im_yz); 

		pcl::toPCLPointCloud2(*seg_point,*incomplete_seg_cloud); //TODO: replace seg_point with complete_point

		//TODO uncomment for cloud completion
		completePointCloud(seg_point,complete_point);

		//TODO uncomment for normal calculation
		computeFeatures(complete_point);
		getNormals(complete_point,cloud_normals); //TODO: replace seg_point with complete_point

		//TODO uncomment for grasp detection
		findBestGrasp(complete_point,cloud_normals); //TODO: replace seg_point with complete_point

		// pcl::PointXYZ -> pcl::PointCloud2
		pcl::toPCLPointCloud2(*complete_point,*seg_cloud); //TODO: replace seg_point with complete_point
		RCLCPP_INFO_STREAM(this->get_logger(), "SEGMENTED PCL size " << seg_cloud->width * seg_cloud->height); 

		// pcl::PointCloud2 -> sensor_msg
		sensor_msgs::msg::PointCloud2 seg_msg,incomplete_seg_msg;
		pcl_conversions::fromPCL(*seg_cloud,seg_msg);
		pcl_conversions::fromPCL(*incomplete_seg_cloud,incomplete_seg_msg);
		complete_publisher_->publish(seg_msg);
		incomplete_publisher_->publish(incomplete_seg_msg);
	}

	void computeFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud){
		int pcl_size = point_cloud->size();

		_mean = pcl::PointXYZ(0.0,0.0,0.0);

		bool is_valid = false;

		for(auto point: point_cloud->points){
			pcl::PointXYZ point_im = pcl::transformPoint(point,transform_inv_mat);
			if(!is_valid){
				_min_x = point.x;
				_max_x = point.x;
				_min_y = point.y;
				_max_y = point.y;
				_min_im_z = point_im.z;
				_min_im_xz = point_im.x/point_im.z;
				_min_im_yz = point_im.y/point_im.z;
				_max_im_xz = point_im.x/point_im.z;
				_max_im_yz = point_im.y/point_im.z;
				is_valid = true;
			}
			else{
				_min_x = std::min(_min_x,point.x);
				_max_x = std::max(_max_x,point.x);
				_min_y = std::min(_min_y,point.y);
				_max_y = std::max(_max_y,point.y);
				_min_im_z = std::min(_min_im_z,point_im.z);
				_min_im_xz = std::min(_min_im_xz,point_im.x/point_im.z);
				_min_im_yz = std::min(_min_im_yz,point_im.y/point_im.z);
				_max_im_xz = std::max(_max_im_xz,point_im.x/point_im.z);
				_max_im_yz = std::max(_max_im_yz,point_im.y/point_im.z);
			}
			_mean.x += point.x;
			_mean.y += point.y;
			_mean.z += point.z;
		}

		if(pcl_size>0){
			_mean.x /= pcl_size;
			_mean.y /= pcl_size;
			_mean.z /= pcl_size;
		}
	}


	void completePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& incomplete_points,pcl::PointCloud<pcl::PointXYZ>::Ptr& complete_points)
	{
		int input_points = incomplete_points->size();
		float min_score = input_points*_visibility_score/5.0f;
		// calculate all possible planes
		std::vector<std::vector<float>> planes;
		for(float theta = _min_theta;theta<=_max_theta;theta+=(_max_theta-_min_theta)/theta_steps){
			for(float x = _min_x;x<=_max_x;x+=(_max_x-_min_x)/x_steps){
				for(float y = _min_y;y<=_max_y;y+=(_max_y-_min_y)/y_steps){
					std::vector<float> plane_i= {x, y, theta, 0.0};
					planes.push_back(plane_i);
				}
			}
		}
		//for all planes compute metric
		std::vector<float> best_symmetry_plane = {(_max_x+_min_x)/2.0f,(0.06f+_max_y+_min_y)/2.0f,(_max_theta+_min_theta)/2.0f,0.0};

		kd->setInputCloud(incomplete_points);

		for(int i=0;i<int(planes.size());i++){
			for(auto point: incomplete_points->points){
				if(planes[i][3]<min_score) break;
				evaluate_plane(planes[i],point);
			}
			// RCLCPP_INFO_STREAM(this->get_logger(), "Plane" << planes[i][0]<<" "<<planes[i][1]<<" "<<planes[i][2]<<" "<<planes[i][3]); 
		}

		//TODO write logic to find best symmetry plane (max score)
		//std::sort(planes.begin(),planes.end(), sortcol(3));
		std::sort(planes.begin(), planes.end(), sortcol);

		best_symmetry_plane[0] = planes[0][0];
		best_symmetry_plane[1] = planes[0][1];
		best_symmetry_plane[2] = planes[0][2];

		// for(auto plane: planes){
		// 	// std::cout<<"Plane: "<<plane<<"/n" ;
		// 	RCLCPP_INFO_STREAM(this->get_logger(), "Plane" << plane[0]<<" "<<plane[1]<<" "<<plane[2]<<" "<<plane[3]); 
		// }

		//complete the pointcloud using the best plane
		*complete_points = *incomplete_points;
		for(auto point: incomplete_points->points){
			complete_points->push_back(mirror_point(point,best_symmetry_plane));
		}
	}

	void evaluate_plane(std::vector<float>& plane, const pcl::PointXYZ& point){
		//Logic
		float score = 0.0;
		pcl::PointXYZ mirrored_point = mirror_point(point,plane);
		pcl::PointXYZ point_im = pcl::transformPoint(mirrored_point,transform_inv_mat);
		float xz = point_im.x/point_im.z;
		float yz = point_im.y/point_im.z;
		float dist = 1000.0f ;
	
		float err = 0.0f;
		
		int K = 1 ;
		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);
		if ( kd->nearestKSearch (mirrored_point, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
		{
			dist = std::sqrt(pointKNNSquaredDistance[0]) ;
		}

		if(dist <= _min_dist) return;

		if((point_im.z < _min_im_z) ||  xz > _max_im_xz + err || (yz > _max_im_yz + err) || (xz < _min_im_xz - err) || (yz < _min_im_yz - err)) {
			score = _visibility_score ;
		}
		else{
			score = std::max((dist-(dist*dist)/_dist_threshold)*4/_dist_threshold, -1.0f);
			// std::cout<<dist<<" "<<score<<std::endl;
			// score =  std::max((_dist_threshold - dist)/_dist_threshold,-1.0f);
		}
		plane[3] += score;
	}

	pcl::PointXYZ mirror_point(const pcl::PointXYZ& point,const std::vector<float>& plane){

		pcl::PointXYZ mirrored_point(point);
		float x_star, y_star;
		float x,y,theta;
		x = plane[0];
		y = plane[1];
		theta = plane[2];
		
		float x_prime = point.x;
		float y_prime = point.y;
		float dx = x_prime-x;
		float dy = y_prime-y;
		float alpha = std::atan2((y_prime-y), (x_prime-x));
		
		float r = std::sqrt(dx*dx+dy*dy);
		x_star = r*std::cos(2*theta-alpha);
		y_star = r*std::sin(2*theta-alpha);

		mirrored_point.x = x_star;
		mirrored_point.y = y_star;

		return mirrored_point;
	}


	void getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr& complete_points,pcl::PointCloud<pcl::Normal>::Ptr& normals)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(complete_points);

		ne.setSearchMethod(KDtree);
		ne.setRadiusSearch(0.05);
		ne.setViewPoint(_mean.x,_mean.y,_mean.z);
		ne.compute(*normals);

		// RCLCPP_INFO_STREAM(this->get_logger(), "Normals size " << seg_cloud->size()); 

		// TODO Visualization
		// if(normals_valid) bool err = viewer->removePointCloud("cloud");

		// viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(complete_points, normals);
		// normals_valid = 1;

		// viewer->spinOnce(100);

		publish_marker(complete_point,normals);
		
	}

	void publish_marker(pcl::PointCloud<pcl::PointXYZ>::Ptr& complete_points,pcl::PointCloud<pcl::Normal>::Ptr& normals){
		visualization_msgs::msg::MarkerArray marker_array;
		
		for (size_t i = 0; i < complete_points->size(); ++i) {
			visualization_msgs::msg::Marker marker;

			marker.header.frame_id = "world";
			marker.header.stamp = this->now();
			marker.ns = "normals";
			marker.id = i;
			marker.type = visualization_msgs::msg::Marker::ARROW;
			marker.action = visualization_msgs::msg::Marker::ADD;

			// Position
			marker.pose.position.x = complete_points->points[i].x;
			marker.pose.position.y = complete_points->points[i].y;
			marker.pose.position.z = complete_points->points[i].z;

			// Orientation (from normal)
			Eigen::Vector3d normal_vector(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
			Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), normal_vector);
			marker.pose.orientation.x = q.x();
			marker.pose.orientation.y = q.y();
			marker.pose.orientation.z = q.z();
			marker.pose.orientation.w = q.w();

			// Scale
			marker.scale.x = 0.05 / 4.0; // 
			marker.scale.y = 0.005 / 4.0; // 
			marker.scale.z = 0.005 / 4.0; //

			// Color
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
			marker.color.a = 1.0;

			marker_array.markers.push_back(marker);
		}
		marker_publisher_->publish(marker_array);
	}

	void findBestGrasp(pcl::PointCloud<pcl::PointXYZ>::Ptr& complete_points,pcl::PointCloud<pcl::Normal>::Ptr& normals)
	{
		pcl::PointCloud<pcl::PointXYZ> contacts;
		pcl::PointXYZ contact1(0.0,0.0,0.0);
		pcl::PointXYZ contact2(1.0,1.0,1.0);
		pcl::Normal contact1_normal;

		//TODO Calculate Best Grasp contacts ( use  _mean if required)

		// create a KDTree for the points

		kd->setInputCloud(complete_points);

		// get the nearest point to the _mean in KDTree using Knearest function in KDTree and save it in contact1

		int K = 1 ;
		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);
		pcl::PointXYZ nearest_point;
		if ( kd->nearestKSearch (_mean, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
		{
			contact1 = complete_points->points[pointIdxKNNSearch[0]];
			contact1_normal = normals->points[pointIdxKNNSearch[0]];
		}

		double min_angle = 6.24;

		for (size_t i = 0; i < complete_points->points.size(); ++i)
		{
			pcl::PointXYZ current_point = complete_points->points[i];
			pcl::Normal current_normal = normals->points[i];

			// 
			pcl::PointXYZ vector_to_current_point;
			vector_to_current_point.x = current_point.x - contact1.x;
			vector_to_current_point.y = current_point.y - contact1.y;
			vector_to_current_point.z = current_point.z - contact1.z;

			// angle calculation
			double dot_product = (current_normal.normal_x * vector_to_current_point.x +
								current_normal.normal_y * vector_to_current_point.y +
								current_normal.normal_z * vector_to_current_point.z)*-1;
			
			double vector_magnitude = sqrt(vector_to_current_point.x * vector_to_current_point.x +
										vector_to_current_point.y * vector_to_current_point.y +
										vector_to_current_point.z * vector_to_current_point.z);
			// double normal_magnitude = sqrt(current_normal.normal_x * current_normal.normal_x +
			// 							current_normal.normal_y * current_normal.normal_y +
			// 							current_normal.normal_z * current_normal.normal_z);
			double angle = acos(dot_product / (vector_magnitude )); //* normal_magnitude

			// angle calculation
			dot_product = contact1_normal.normal_x * vector_to_current_point.x +
								contact1_normal.normal_y * vector_to_current_point.y +
								contact1_normal.normal_z * vector_to_current_point.z;
			angle += acos(dot_product / (vector_magnitude )); //* normal_magnitude

			// minimum angle checks
			if (angle < min_angle)
			{
				min_angle = angle;
				contact2 = current_point;
			}
		}

		// publish contacts
		contacts.push_back(contact1);
		contacts.push_back(contact2);
		pcl::toPCLPointCloud2(contacts,*contact_cloud);
		// sensor_msgs::msg::PointCloud2 contact_msg;
		// pcl_conversions::fromPCL(*contact_cloud,contact_msg);
		// contact_msg.header.frame_id = "camera_link";

		visualization_msgs::msg::MarkerArray contact_msg;
		
		visualization_msgs::msg::Marker marker;

		marker.header.frame_id = "world";
		marker.header.stamp = this->now();
		marker.ns = "contacts";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.action = visualization_msgs::msg::Marker::ADD;

		// Scale
		marker.scale.x = 0.05 / 4.0; // 
		marker.scale.y = 0.05 / 4.0; // 
		marker.scale.z = 0.05 / 4.0; //

		// Color
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;

		// Position
		marker.pose.position.x = contact1.x;
		marker.pose.position.y = contact1.y;
		marker.pose.position.z = contact1.z;

		contact_msg.markers.push_back(marker);

		marker.id = 1;

		marker.pose.position.x = contact2.x;
		marker.pose.position.y = contact2.y;
		marker.pose.position.z = contact2.z;

		contact_msg.markers.push_back(marker);

		contact_publisher_->publish(contact_msg);
	}

// member variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr complete_publisher_,incomplete_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_,contact_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr input_cloud, filtered_cloud, seg_cloud, contact_cloud, incomplete_seg_cloud;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_point, complete_point, seg_point_world;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	//TODO uncomment this for viewer
	// pcl::visualization::PCLVisualizer::Ptr viewer;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr KDtree;


	Eigen::Matrix4f transform_mat;
	Eigen::Affine3f transform_inv_mat;
    float callback_sec;
	bool normals_valid;

	float _min_x, _max_x, _min_y, _max_y, _min_theta, _max_theta;
	float x_steps,y_steps,theta_steps;
	float _min_im_z, _min_im_yz, _max_im_yz, _min_im_xz, _max_im_xz;
	float _visibility_score, _dist_threshold, _min_dist;
	
	pcl::PointXYZ _mean;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd;
};

int main(int argc, char* argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectSegmentation>());
	rclcpp::shutdown();
	return 0;
}
