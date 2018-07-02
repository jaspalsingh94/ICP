/*
	Jaspal Singh
	Project 2
	3d Photography
	Professor Stamos

	This program takes 2 point clouds and registers them using Iterative Closest Point. The error function is defined for both point-to-point and point-to-plane in the PCL class.
*/

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "Pcl_reg.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

using namespace Eigen;
using std::cout;
using std::cin;
using std::endl;
using std::string;


int main(int argc, char **argv){

	if(argc != 8)
	{
		printf("Usage: %s input_1.ptx input_2.ptx input_3.ptx out_pt_1.ptx out_plane_1.ptx out_pt_2.ptx out_plane_2.ptx\n", argv[0]);
                return 0;
	}

	const string input_1(argv[1]); //gh16.ptx
	const string input_2(argv[2]);	//gh17.ptx
	const string input_3(argv[3]);	//gh23.ptx
	const string out_pt_1(argv[4]);
	const string out_plane_1(argv[5]);
	const string out_pt_2(argv[6]);
	const string out_plane_2(argv[7]);

	PCL_REG first_pcl(input_1); //gh16
	PCL_REG second_pcl_pt(input_2); //gh17
		
	Matrix3f init_rot;
	init_rot << 0.999953, 0.004953, 0.008324, 
				   -0.003077, 0.977294, -0.211865,
				   -0.009184, 0.211829, 0.977264;
	Vector3f init_translation;
	init_translation << -0.000003, 0.001217, 0.017408;
	

	//Apply the initial transformation to bring the point clouds near each other.
	second_pcl_pt.apply_example_rot(init_rot, init_translation);

	//icp(other_pcl, pt_to_pt distance or pt_to_plane, threshold_for_error, threshold_for_point_correspondence, how_much_to_decrement_the point_corresp_threshold)
	second_pcl_pt.icp(first_pcl, true, 0.24, 1.5, 0.25); 
	second_pcl_pt.write_to_file(out_pt_1, 30);

	// //Rerun icp with point_to_plane distance. 
	PCL_REG second_pcl_plane(input_2);
	second_pcl_plane.apply_example_rot(init_rot, init_translation);
	second_pcl_plane.icp(first_pcl, false, 0.26, 1.7, 0.25);
	second_pcl_plane.write_to_file(out_plane_1, 50);

	PCL_REG second_pcl_2_pt(input_2); // gh17
	PCL_REG second_pcl_2_plane(input_2);	//gh17
	PCL_REG third_pcl(input_3);	//gh23


	init_rot << 0.769006,  0.300208,  0.564362,
				-0.340899,  0.939441, -0.035215,
				-0.540756, -0.165310,  0.824776;
	init_translation << 7.214807, - 1.341283, - 2.291782;

	second_pcl_2_pt.apply_example_rot(init_rot, init_translation);
	second_pcl_2_plane.apply_example_rot(init_rot, init_translation);

	second_pcl_2_pt.icp(third_pcl, true, 0.25, 0.6, 0.1);
	second_pcl_2_pt.write_to_file(out_pt_2, 80);

	second_pcl_2_plane.icp(third_pcl, false, 0.35, 1, 0.1);
	second_pcl_2_plane.write_to_file(out_plane_2, 100);

	return 0;
}
