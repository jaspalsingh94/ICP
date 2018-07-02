//Definition of PCL_REG class


#include <fstream>
#include <eigen3/Eigen/Dense>
#include "Pcl_reg.h"
#include <string>
#include <cmath>
#include <time.h>

using namespace Eigen;
using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::cerr;
using std::fstream;
using std::ofstream;
using namespace std;
size_t points_for_corresp = 1000;

//Constructor with file name.
PCL_REG::PCL_REG(const string file_name): file_name(file_name), this_corresp_points(points_for_corresp), other_corresp_points(points_for_corresp), 
										other_corresp_normals(points_for_corresp), dist_between_points(points_for_corresp)
{
	bool file_loaded = load_file(this->file_name);
	if(!file_loaded)
	{
		cerr << "Unable to load the file. \n";
	}
} 

//Copy constructor.
PCL_REG::PCL_REG(const PCL_REG &pcl_2, const string new_file_name) : file_name(new_file_name), centeroid({0, 0, 0}), other_center({0, 0, 0}), this_corresp_points(points_for_corresp), 
															other_corresp_points(points_for_corresp), other_corresp_normals(points_for_corresp), dist_between_points(points_for_corresp)
{
	scan_lines = pcl_2.get_scan_lines();
	pts_per_scanline = pcl_2.get_pts_per_scanline();
	//Copy the transformation lines
	string pcl_2_transform_lines = pcl_2.get_transform_lines();
	for(int i = 0; i < 8; i++)
	{ transform_lines[i] = pcl_2_transform_lines[i]; }

	//Create the 3-dimensional array for points and copy the points from PCL_2. Also create the 2 dimensioanl array of Vectors and copy the normals.
	float*** pts_pcl_2 = pcl_2.get_points();
	Vector3f** normals_pcl_2 = pcl_2.get_normal_vectors();

	points = new float**[scan_lines];
	normal_vectors = new Vector3f*[scan_lines];

	for(size_t i = 0; i < scan_lines; i++)
	{	
		points[i] = new float*[pts_per_scanline];
		normal_vectors[i] = new Vector3f[pts_per_scanline];

		for(size_t j = 0; j < pts_per_scanline; j++)
		{
			points[i][j] = new float[4];
			normal_vectors[i][j] = normals_pcl_2[i][j];
			for(int k = 0; k < 4; k++)
			{
				points[i][j][k] = pts_pcl_2[i][j][k];
			}
		}
	}
	cout << "Done copying data into the current object. \n";
}

// Loads the file into points and calls the compute_normals to compute the normals. After computing the normals, 
// decenteralize the entire point cloud. 
bool PCL_REG::load_file(const string file_name)
{
	fstream pclfile;
	pclfile.open(file_name.c_str());
	if(pclfile)
	{
		pclfile >> scan_lines;
		pclfile >> pts_per_scanline;

		//Store the 8 transformation lines
		for(size_t i = 0; i < 8; i++)
		{	getline(pclfile, transform_lines[i]);	}
		
		//Define the 3-dimensional array for the point data.
		points = new float**[scan_lines];
		for(size_t i = 0; i < scan_lines; i++)
		{
			points[i] = new float*[pts_per_scanline];
			for(size_t j = 0; j < pts_per_scanline; j++)
			{
				points[i][j] = new float[4];
			}
		}

		//Define the 2 dimensional array for normals.
		normal_vectors = new Vector3f*[scan_lines];
		for(size_t i =0; i < scan_lines; i++)
		{	normal_vectors[i] = new Vector3f[pts_per_scanline]; }

		//Read the pointcloud into points. Intensity gets set to 0.
		float dummy;
		for(size_t i = 0; i < scan_lines; i++)
		{
			for(size_t j =0; j < pts_per_scanline; j++)
			{
				normal_vectors[i][j] << 0, 0, 0; //Initialize all the normal vectors to 0.
				for(int k = 0; k < 4; k++)
				{
					if(k != 3)
					{ pclfile >> points[i][j][k]; } 
					else
					{	pclfile >> dummy;
						points[i][j][k] = 0; }
				}
			}
		}
		compute_normals();
		pclfile.close();
		return true;

	}
	else
	{
		cerr << "Unable to open the Point Cloud\n";
		return false;
	}
}

void PCL_REG::compute_normals()
{
	cout << "Computing normals... \n";

	size_t num_of_pts = 0;
	size_t normals_not_computed = 0;

	Matrix3f cov_mat; //Covariance matrix for normal computation.
	Vector3f eigen_vals; //Vector for storing eigenvalues.
	Vector3f points_decent[3][3]; //3x3 matrix of decentralized vectors for normal computation.
	Matrix3f eigen_vecs; //Matrix for storing eigenvectors.

	cov_mat << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	/* For each point, we take a neighborhood of 3x3 points, decentralize them by removing the center, and then 
	   compute the covariance of each point (which will be a 3x3 matrix). Add the covariance matrices for each point and then compute the
	   eigenalues and the eigenvectors of the covariance matrix. Eigenvector corresponding to the smallest eigenvalue is 
	   the normal we are going to keep.
	*/
	for(size_t i = 1; i< scan_lines-1; i++){	
		for(size_t j = 1; j < pts_per_scanline-1; j++)
		{
			cov_mat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
			if((points[i][j][0] != 0) || (points[i][j][1] != 0) || (points[i][j][2] != 0)){
				num_of_pts++;
				int m = i-1;
				int n = j-1;
				float center[3] = {0, 0, 0};
				float divider = 0;
				//compute the center
				for(int k = 0; k < 3; k++){
					for(int l = 0; l < 3; l++){
						if((points[m+k][n+l][0] != 0) || (points[m+k][n+l][1] != 0) || (points[m+k][n+l][2] != 0)){
							center[0] += points[m+k][n+l][0];
							center[1] += points[m+k][n+l][1];
							center[2] += points[m+k][n+l][2];
							divider++;							
						}
					}
				}
				if(divider != 0){
					center[0] /= divider;
					center[1] /= divider;
					center[2] /= divider;			
				}

				//Subtract each point from its center and compute the covariance by P * P.Transpose() -> symmetric matrix
				for(int k = 0; k < 3; k++){
					for(int l = 0; l < 3; l++){
						points_decent[k][l] << (points[m+k][n+l][0] - center[0]),
												(points[m+k][n+l][1] - center[1]),
												(points[m+k][n+l][2] - center[2]);
						cov_mat += points_decent[k][l] * points_decent[k][l].transpose();
					}
				} // End of covariance computation loop.

				/* For a symmetric matrix, the SelfAdjointEigenSolver gives the eigenvalues in a sorted order. We take the eigenvector corresponding to 
				   the smallest eigenvalue, if one exists. Throw away eigenvalues larger than the threshold. 
				*/

				SelfAdjointEigenSolver<Matrix3f> eg(cov_mat);
				//Check to see if the eigensolver converges. If it doesn't, normal stays 0.
				if(eg.info() == Success)
				{
					eigen_vals = eg.eigenvalues();
					eigen_vecs = eg.eigenvectors();
					//Take the non-zero eigenvalue less than 100.

					if((eigen_vals(0) > 0) && (eigen_vals(0) < 100))
					{	normal_vectors[i][j] << eigen_vecs(0, 0), eigen_vecs(1, 0), eigen_vecs(2, 0); }
					else if((eigen_vals(1) > 0) && (eigen_vals(1) < 100))
					{	normal_vectors[i][j] << eigen_vecs(0, 1), eigen_vecs(1, 1), eigen_vecs(2, 1); }
					else if((eigen_vals(2) > 0) && (eigen_vals(2) < 100))
					{ 	normal_vectors[i][j] << eigen_vecs(0, 2), eigen_vecs(1, 2), eigen_vecs(2, 2); }
					else { normals_not_computed++; }
				}
				else { normals_not_computed++; }
			} 
		}
	}//Done computing normals.
	cout << (num_of_pts - normals_not_computed) << " normals computed out of " << num_of_pts << " points." << endl;
}

//Applies the rotation matrix to the entire point cloud. this->points
void PCL_REG::apply_rot(Vector3f translation)
{	
	Vector3f this_point;
	for(size_t i = 0; i < scan_lines; i++){
		for(size_t j = 0; j < pts_per_scanline; j++)
		{
			this_point << points[i][j][0], points[i][j][1], points[i][j][2];
			this_point = rot_mat*this_point;
			this_point += translation;
			points[i][j][0] = this_point[0];
			points[i][j][1] = this_point[1];
			points[i][j][2] = this_point[2];

			normal_vectors[i][j] = rot_mat*normal_vectors[i][j];
		}
	}
}

Vector3f PCL_REG::compute_translation()
{	
	Vector3f translation;
	translation << 0, 0, 0;
	translation = other_center - (rot_mat*centeroid);
	return translation;
}



void PCL_REG::icp(const PCL_REG &pcl_2, bool pt_to_pt, float error_threshold, float pt_corresp_threshold, float pt_threshold_decrementor)
{
	cout << "______________________________Starting ICP________________________________ \n";

	float*** other_points = pcl_2.get_points();
	size_t other_scan_lines = pcl_2.get_scan_lines();
	size_t other_pts_per_scan = pcl_2.get_pts_per_scanline();
	Vector3f** other_normals = pcl_2.get_normal_vectors();
	// Vector3f other_center = pcl_2.get_centroid();
	Vector3f translation;

	// Build the 3 dimensional k-d tree from the other point cloud data. 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = other_scan_lines-2; //Don't use the points on the boundary
	cloud->height = other_pts_per_scan-2;
	cloud->points.resize(cloud->width * cloud->height);
	size_t idx;
	for(size_t i = 1; i < other_scan_lines-1; i++){
		for(size_t j = 1; j < other_pts_per_scan-1; j++){
			//Need to map i,j -> 0, 1, ... width*height.
			// map (i,j) -> (i-1)*other_pts_per_scan-1 + (j-1) 
			idx = (i-1)*(other_pts_per_scan-2) + (j-1);
			cloud->points[idx].x = other_points[i][j][0];
			cloud->points[idx].y = other_points[i][j][1];
			cloud->points[idx].z = other_points[i][j][2];
		}
	}

	//Feeding the entire data into k-d tree at once gaurantees the tree being balanced-> O(lg n) for each search.
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// Make corresponding vectors:
	if(pt_to_pt){
		make_corresponding_lists(kdtree, cloud, other_scan_lines, other_pts_per_scan, other_normals, pt_corresp_threshold, true);
	}
	else
	{
		make_corresponding_lists(kdtree, cloud, other_scan_lines, other_pts_per_scan, other_normals, pt_corresp_threshold, false);
	}
	float error;
	int iteration = 1;
	do
	{
		// cout << "Running iteration # " << iteration << endl; 
		/*
			In each iteration, compute the Rotation matrix using the lists of corresponding points by calling minimize_error. Then, apply the rotation to 
			the entire point cloud. Rebuild the corresponding lists with different points. Compute the error based on the new correspondences. Stop once the 
			error is below the threshold. In the next iteration, the correspondence list computed in this iteration will be used to compute the rotation. 
		*/
		if(pt_to_pt)
		{
			minimize_error();
			translation = compute_translation();
			apply_rot(translation);				
			make_corresponding_lists(kdtree, cloud, other_scan_lines, other_pts_per_scan, other_normals, pt_corresp_threshold-(iteration*pt_threshold_decrementor), true);
		}
		else
		{
			translation = minimize_error_pt_to_plane();
			apply_rot(translation);
			make_corresponding_lists(kdtree, cloud, other_scan_lines, other_pts_per_scan, other_normals, pt_corresp_threshold-(iteration*pt_threshold_decrementor), false);
		}
		error = compute_error();
		cout << "For iteration: " << iteration << ", the error is: " << error << endl;
		iteration++;
		// cout << "Current rotation matrix: " << rot_mat << endl;
	} while(error > error_threshold && iteration<15);
	cout << "Done computing the rotation for registration.\n";

}


// Makes 2 vectors of length 1000 where the points in the vectors correspond to each other. Correspondence is done via point-to-point distance or point-to-plane distance.
void PCL_REG::make_corresponding_lists(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t other_scans, size_t other_pts_per_scan, Vector3f** other_normals, float pt_dist, bool pt_to_pt)
{	
	Vector3f point, other_point;  // Corresponding points that will be saved in the vector.
	pcl::PointXYZ search_point; //PCL point for the nearest neighbor search.
	vector<int> idx_from_NNSearch(1); // Vector required for NN search: stores the indices of neighbors
	vector<float> squared_dist_NNSearch(1); //Stores the distance of the found neighbors
	size_t row, col, other_row, other_col;
	srand(time(0));
	int not_added_pts = 0;
	centeroid << 0, 0, 0;
	other_center << 0, 0, 0;
	for(size_t i = 0; i < points_for_corresp; i++){
		point << 0, 0, 0;
		//Randomly pick a non-zero point
		while(!not_zero_vector(point))
		{	
			row = (rand() % (scan_lines - 2) + 1);
			col = (rand() % (pts_per_scanline - 2) + 1);
			point << points[row][col][0], points[row][col][1], points[row][col][2];
			search_point.x = point(0);
			search_point.y = point(1);
			search_point.z = point(2);
		}
		// nearestKsearch returns the number of neighbors it found -> in our case its 0 or 1 since k=1.
		if( kdtree.nearestKSearch(search_point, 1, idx_from_NNSearch, squared_dist_NNSearch) > 0)
		{	
			int index = idx_from_NNSearch[0];
			float dist = squared_dist_NNSearch[0];
			other_point << cloud->points[index].x, cloud->points[index].y, cloud->points[index].z;
			other_row = (index/(other_pts_per_scan-2)) + 1; // compute the row index to look up the normal: idx was computed via [i+j-2].
			other_col = (index % (other_pts_per_scan-2)) + 1; 
			//Add the points if they have similar normal and the distance is less than the threshold.
			if((sqrt(dist) < pt_dist) && (similar_normals(normal_vectors[row][col], other_normals[other_row][other_col])))
			{	
				this_corresp_points[i] = point;
				other_corresp_points[i] = other_point;
				dist_between_points[i] = dist;
				other_corresp_normals[i] = other_normals[other_row][other_col];
				centeroid += point;
				other_center += other_point;
				// cout << "The original point is : " << point(0) << ' ' << point(1) << ' ' << point(2) << endl;
				// cout << "The closest point is: " << other_point(0) << ' ' << other_point(1) << ' ' << other_point(2) << endl;
			}
			else
			{	i--;
				not_added_pts++;
				continue; }
		}
		else
		{	i--;
			not_added_pts++;
			continue; }
	}
	centeroid = centeroid/points_for_corresp;
	other_center = other_center/points_for_corresp;
	if(pt_to_pt)
	{	decenteralize_correspondences();  }	
	cout << not_added_pts << " points were considered as corresponding, but were thrown away because they had bad correspondences.\n";
}

/* Computes the Rotation matrix for corresponding points using the closed form solution for rigid-body transformation (HORN).
   Compute the covariance matrix first. This matrix has as its eigenvector the rotation axis that maps the 2 point sets into each other.
   Then do singular value decomposition on the matrix. H = UAV .We need the left and the rigth singular vectors. 
   The rotation matrix is: R = V * U.transpose().  
*/
void PCL_REG::minimize_error()
{	
	Matrix3f cov_mat;
	cov_mat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	//Compute the covariance matrix of the 2 point sets.
	for(size_t i = 0; i < points_for_corresp; i++)
	{	cov_mat += other_corresp_points[i] * this_corresp_points[i].transpose(); }
	// Compute the left and the right singular vectors of the covariance matrix. 
	JacobiSVD<MatrixXf> svd(cov_mat, ComputeThinU | ComputeThinV);
	rot_mat = svd.matrixV() * svd.matrixU().transpose();

}

Vector3f PCL_REG::minimize_error_pt_to_plane()
{
	Vector3f c_vec;
	VectorXf c_n_vec(6);
	VectorXf x_vec(6);
	VectorXf b_vec(6); 
	MatrixXf cov_mat_plane(6, 6);
	Vector3f translation;

	c_vec << 0, 0, 0;
	c_n_vec << 0, 0, 0, 0, 0, 0;
	x_vec << 0, 0, 0, 0, 0, 0;
	b_vec << 0, 0, 0, 0, 0, 0;
	cov_mat_plane << 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0;

	/*
		The 6-dim vector c_n is (cx, cy, cz, nx, ny, nz).
		c_vec is (p x n).
		The equation for the computation of the rotation and translation is 
			[(c_n)(c_n).transpose][r t] = [c_n]*(p-q).n
			where r = (a, b, c) and t = (tx, ty, tz) 
		So need to solve Ax = b ---> x = (((A.transpose)*A) ^-1) * (A.transpose)b
	*/

	for(size_t i = 0; i < points_for_corresp; i++)
	{
		c_vec = this_corresp_points[i].cross(other_corresp_normals[i]);
		c_n_vec << c_vec, other_corresp_normals[i];
		b_vec = c_n_vec * (this_corresp_points[i] - other_corresp_points[i]).dot(other_corresp_normals[i]);
		cov_mat_plane = c_n_vec * c_n_vec.transpose();

		JacobiSVD<MatrixXf> lls(cov_mat_plane, ComputeThinU | ComputeThinV);
		x_vec = lls.solve(b_vec);

		translation << x_vec(3), x_vec(4), x_vec(5);

		rot_mat << 1, -x_vec(2), x_vec(1),
					x_vec(2), 1, -x_vec(0),
					-x_vec(1), x_vec(0), 1;

	}
	return translation;
}


//Sum of the distances of corresponding points. 
float PCL_REG::compute_error()
{	
	float error = 0;
	for(size_t i = 0; i < points_for_corresp; i++)
	{	error += sqrt(dist_between_points[i]); }
	return error/points_for_corresp;
}


bool PCL_REG::write_to_file(const string file_name, int intensity)
{
	ofstream pcl_out_file(file_name);
	if(pcl_out_file.is_open())
	{
		pcl_out_file << scan_lines << endl;
		pcl_out_file << pts_per_scanline;

		for(int i = 0; i < 8; i++)
		{
			pcl_out_file << transform_lines[i] << endl;
		}

		for(size_t i = 0; i < scan_lines; i++){
			for(size_t j = 0; j < pts_per_scanline; j++)
			{
				pcl_out_file << points[i][j][0] << ' ' << points[i][j][1] << ' ' << points [i][j][2] << ' ' << intensity << endl;
			}
		}
		pcl_out_file.close();
		cout << "Wrote to " << file_name << " file.\n";
		return true;
	}
	else{
		cerr << "Unable to write to file.\n";
		return false;
	}
}

void PCL_REG::decenteralize_correspondences()
{
	for(size_t i = 0; i < points_for_corresp; i++)
	{
		this_corresp_points[i] -= centeroid;
		other_corresp_points[i] -= other_center;
	}
}

//Compares the given normals.
bool PCL_REG::similar_normals(Vector3f &norm_1, Vector3f &norm_2) 
{
	if(abs(norm_1.dot(norm_2)) > 0.965) //.965 for first
		return true;
	else 
		return false;
}


// checks if the vector is not zero.
bool PCL_REG::not_zero_vector(Vector3f & vec) const 
{
	if((vec(0) == 0) && (vec(1) == 0) && (vec(2) == 0))
		return false;
	else
		return true; 
}


void PCL_REG::apply_example_rot(Matrix3f & example_rot, Vector3f translation)
{	
	rot_mat = example_rot;
	apply_rot(translation);
}


//getter for scan_lines
size_t PCL_REG::get_scan_lines() const
{ return this->scan_lines; }

//getter for pts_per_scan_line
size_t PCL_REG::get_pts_per_scanline() const
{ return this-> pts_per_scanline; }

//getter for the actual point data
float*** PCL_REG::get_points() const
{ return this->points; }

//getter for the computed normals
Vector3f** PCL_REG::get_normal_vectors() const
{ return this->normal_vectors; }

//getter for the transformation lines
string PCL_REG::get_transform_lines() const 
{ return this->transform_lines[0]; }

Vector3f PCL_REG::get_centroid() const
{ return this->centeroid; }

//Destructor for the class. 
PCL_REG::~PCL_REG()
{
	//free the memory
	for(size_t i = 0; i < scan_lines; i++)
	{
		delete [] normal_vectors[i];
		for(size_t j = 0; j < pts_per_scanline; j++)
		{
			delete [] points[i][j];
		}
		delete [] points[i];
	}
	delete [] points;
	delete [] normal_vectors;
	cout << "Memory deallocated successfully." << endl;
}
