#ifndef PCL_REG_H
#define PCL_REG_H

/*
  Class for PCL:
	Stores the point cloud and defines the functions to register the point cloud.	
*/

#include <fstream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <ctime>

using namespace Eigen;
using std::string;
using std::vector;


class PCL_REG
{

public:
	bool load_file(const string file_name);
	bool write_to_file(const string file_name, int intensity);
	PCL_REG(): scan_lines(0), pts_per_scanline(0), points(0), file_name(" "), normal_vectors(0), transform_lines({"","","","","","","",""}), 
			centeroid({0, 0, 0}), other_center({0, 0, 0}), this_corresp_points(50), other_corresp_points(50), other_corresp_normals(50), dist_between_points(50) {}
	PCL_REG(const string file_name); //Constructor with the file
	PCL_REG(const PCL_REG &pcl_2, const string new_filename); //Copy constructor (need to pass the new name of the file.)

	~PCL_REG();
	

	size_t get_scan_lines() const;
	size_t get_pts_per_scanline() const;
	float*** get_points() const;
	Vector3f** get_normal_vectors() const;
	string get_transform_lines() const;
	Vector3f get_centroid() const;
	void icp(const PCL_REG &pcl_2, bool pt_to_pt, float error_threshold, float pt_corresp_threshold, float pt_threshold_decrementor); 

	void apply_example_rot(Matrix3f & example_rot, Vector3f translation);




private:
	size_t scan_lines;
	size_t pts_per_scanline;
	float*** points;
	string file_name;
	Vector3f** normal_vectors;
	string transform_lines[8];
	Vector3f centeroid;
	Vector3f other_center;
	Matrix3f rot_mat;
	vector<Vector3f> this_corresp_points;
	vector<Vector3f> other_corresp_points;
	vector<Vector3f> other_corresp_normals;
	vector<float> dist_between_points;

	void compute_normals();
	void apply_rot(Vector3f translation);
	void make_corresponding_lists(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t other_scans, size_t other_pts_per_scan, Vector3f** other_normals, float pt_dist, bool pt_to_pt);
	bool similar_normals(Vector3f &norm_1, Vector3f &norm_2);
	bool not_zero_vector(Vector3f & vec) const;
	void minimize_error();
	float compute_error();
	Vector3f compute_translation();
	void decenteralize_correspondences();
	Vector3f minimize_error_pt_to_plane();
};

#endif
