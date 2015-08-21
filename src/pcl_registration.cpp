/**
@page pcl_registration Модуль совмещения облак точек
В этом модуле выполняется совмещение двух облак точек в единое облако.
На вход исполняемому файлу подаются пути к двум облакам точек в формате ply
\code
./pcl_registration rotate_robot_30.ply rotate_robot_35.ply
\endcode
в этом примере мы планируем совместить изображения, полученные из разных ракурсов.
В результате выполнения, откроется интерактивный графический редактор где будет видно насколько хорошо совместились облака точек
*/
#include "utils.h"
//Измените этот typedef в зависимости от того, с каким облаками точек вы работаете.
//Если у вас облака точек в формате ply содержат только XYZ, то поменяйте на typedef PointType pcl::PointXYZ
typedef pcl::PointXYZRGB PointType;

using namespace utils;

int main(int argc, char *argv[]) {
    pcl::PointCloud<PointType>::Ptr cloud_1_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_1_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr keypoints_1_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_1_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors_1_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::PointCloud<PointType>::Ptr cloud_2_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_2_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr keypoints_2_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_2_ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors_2_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

    if (pcl::io::loadPLYFile<PointType> (argv[1], *cloud_1_ptr) == -1) {
            PCL_ERROR ("Couldn't load the file\n");
            return (-1);
    }
    if (pcl::io::loadPLYFile<PointType> (argv[2], *cloud_2_ptr) == -1) {
            PCL_ERROR ("Couldn't load the file\n");
            return (-1);
    }
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (cloud_1_ptr, "points");
    viz.spin ();

    //Downsample the cloud
    const float voxel_grid_leaf_size = 0.01;
    downsample<PointType>(cloud_1_ptr, voxel_grid_leaf_size, downsampled_1_ptr);
    downsample<PointType>(cloud_2_ptr, voxel_grid_leaf_size, downsampled_2_ptr);

//    //Filtered the clouds
    filtered_pointcloud<PointType>(downsampled_1_ptr);
    filtered_pointcloud<PointType>(downsampled_2_ptr);

    // Compute surface normals
    const float normal_radius = 0.03;
    compute_surface_normals<PointType>(downsampled_1_ptr, normal_radius, normals_1_ptr);
    compute_surface_normals<PointType>(downsampled_2_ptr, normal_radius, normals_2_ptr);

    //Compute keypoints
    detect_keypoints<PointType>(downsampled_1_ptr, keypoints_1_ptr);
    detect_keypoints<PointType>(downsampled_2_ptr, keypoints_2_ptr);
    //visualize_keypoints(downsampled_1_ptr, keypoints_1_ptr);

    //Compute descriptors
    detect_descriptors<PointType>(downsampled_1_ptr, normals_1_ptr, keypoints_1_ptr, descriptors_1_ptr);
    detect_descriptors<PointType>(downsampled_2_ptr, normals_2_ptr, keypoints_2_ptr, descriptors_2_ptr);

    //Find correspondence
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource (descriptors_1_ptr);
    est.setInputTarget (descriptors_2_ptr);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences), remaining_correspondences(new pcl::Correspondences);
    est.determineCorrespondences (*correspondences);
    std::cout << "Found " << correspondences->size () << " correspondences" << std::endl;
    visualize_correspondences<PointType>(downsampled_1_ptr, downsampled_2_ptr, *correspondences);

    //Reject bad correspondence
    pcl::registration::CorrespondenceRejector::Ptr rejector(new pcl::registration::CorrespondenceRejectorDistance());
    boost::static_pointer_cast<pcl::registration::CorrespondenceRejectorDistance>(rejector)->setMaximumDistance(2);
    reject_bad_feature<PointType>(downsampled_1_ptr, normals_1_ptr, downsampled_2_ptr, normals_2_ptr, *correspondences, *remaining_correspondences, rejector);
    std::cout << "Remain after reject: " << remaining_correspondences->size () << " correspondences" << std::endl;

    pcl::registration::TransformationEstimationSVD<PointType, PointType> tSVD;
    Eigen::Matrix4f transformation;
    pcl::PointCloud<PointType>::Ptr result_cloud_ptr(new pcl::PointCloud<PointType>);

    tSVD.estimateRigidTransformation (*downsampled_1_ptr, *downsampled_2_ptr, *remaining_correspondences, transformation);
    pcl::transformPointCloud(*downsampled_1_ptr, *result_cloud_ptr, transformation);
    *downsampled_2_ptr += *result_cloud_ptr;
    visualize_2cloud<PointType>(downsampled_1_ptr, downsampled_2_ptr);
}
