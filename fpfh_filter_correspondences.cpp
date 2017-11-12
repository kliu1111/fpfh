#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	//est_normal.setKSearch(5);
	est_normal.setRadiusSearch(0.05);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	//est_fpfh.setKSearch(5);
	est_fpfh.setRadiusSearch(0.05);
	est_fpfh.compute(*fpfh);
	return fpfh;
}

int main()
{
	clock_t start, end, time;
	start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::io::loadPCDFile<pcl::PointXYZ>("test31.pcd", *source);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("test32.pcd", *target);
	pcl::io::loadPLYFile("left_sub_sam.ply", *source);
	pcl::io::loadPLYFile("right_sub_sam.ply", *target);
    std:vector<int> index;
	pcl::removeNaNFromPointCloud(*source, *source, index);
	pcl::removeNaNFromPointCloud(*target, *target, index);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);


	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//  crude_cor_est.determineCorrespondences(cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is:" << cru_correspondences->size() << endl;

	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);

	float a, shreshold = 90;
	int total = 0;
	for (int i = 0; i < cru_correspondences->size(); i++)
   {
		a = cru_correspondences->at(i).distance;
		if (a <= shreshold)
		{
			correspondences->push_back(cru_correspondences->at(i));
			total++;
		}	

	}
	    cout <<"After filter total correspondences is"<<total << endl;
		Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);

		trans->estimateRigidTransformation(*source, *target, *correspondences, Transform);

		cout << Transform << endl;
       
		end = clock();
		cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;
		pcl::transformPointCloud(*source, *source, Transform);
		pcl::io::savePLYFile("left_sub_reg_right.ply", *source, false);
		

		system("pause");
		return 0;
	}