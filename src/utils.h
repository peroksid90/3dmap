#ifndef UTILS_H
#define UTILS_H
/**
@page Utils Функции для пользователей
Функции для визуализации облак точек, нахождения keypoints, features, парсинга форматов и т.д.
Подробней смотрите \ref utils "namespace utils".
*/

#include <boost/filesystem.hpp>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/io/ply_io.h>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/io/ply_io.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include <string>
#include "pcl/features/pfh.h"
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <stdio.h>
/**
namespace utils
@brief Пространство имен utils
*/
namespace utils {
namespace fs = boost::filesystem;

/**
    @brief Уменьшить размер облака точек используя <a href="http://pointclouds.org/documentation/tutorials/voxel_grid.php">VoxelGrid фильтр</a>
    @param points [in] исходное облако точек
    @param leaf_size [in] размер стороны Voxel
    @param downsampled_out [out] уменьшенное выходное облако точек
*/
template<typename PointType>
void downsample (typename pcl::PointCloud<PointType>::Ptr &points, float leaf_size,
            typename pcl::PointCloud<PointType>::Ptr &downsampled_out) {
    pcl::VoxelGrid<PointType> vox_grid;
    vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud (points);
    vox_grid.filter (*downsampled_out);
}

/**
    @brief Определить <a href="http://pointclouds.org/documentation/tutorials/walkthrough.php#keypoints">keypoints</a> по облаку точек
    @details используется алгоритм <a href="http://docs.pointclouds.org/trunk/classpcl_1_1_s_i_f_t_keypoint.html">SIFT</a>
    @param cloud_ptr [in] исходное облако точек
    @param keypoints_ptr [out] сюда будут записаны найденные keypoints
*/
template<typename PointType>
void detect_keypoints (typename pcl::PointCloud<PointType>::Ptr& cloud_ptr, typename pcl::PointCloud<PointType>::Ptr& keypoints_ptr) {
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    pcl::SIFTKeypoint<PointType, PointType> sift;
    sift.setSearchMethod (tree);
    sift.setInputCloud (cloud_ptr);
    sift.setScales (0.1, 10, 2);
    sift.setMinimumContrast(0.4);
    sift.compute (*keypoints_ptr);
}

/**
    @brief Определить <a href="http://pointclouds.org/documentation/tutorials/walkthrough.php#features">descriptors(features)</a> по облаку точек
    @details используется алгоритм <a href="http://docs.pointclouds.org/trunk/classpcl_1_1_f_p_f_h_estimation.html">FPFH</a>
    @param cloud [in] исходное облако точек
    @param normals [in] нормали, могут быть получены при помощи функции <compute_surface_normals>"()"
    @param keypoints [in] keypoints, могут быть получены при помощи функции <detect_keypoints>"()"
    @param descriptors [out] сюда будут записаны найденные descriptors
*/
template<typename PointType>
void detect_descriptors(typename pcl::PointCloud<PointType>::Ptr cloud, typename pcl::PointCloud<pcl::Normal>::Ptr normals
                        ,typename pcl::PointCloud<PointType>::Ptr keypoints, typename pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors) {
    pcl::FPFHEstimation<PointType, pcl::Normal> fpfh;
    fpfh.setSearchSurface(cloud);
    fpfh.setInputCloud(keypoints);
    fpfh.setInputNormals(normals);
    fpfh.setKSearch (150); //взято от "балды", просто посмотреть как работает
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    fpfh.setSearchMethod(tree);
    fpfh.compute(*descriptors);
}

/**
    @brief Определить <a href="http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation">нормали</a> в облаке точек
    @param points [in] исходное облако точек
    @param normal_radius [in] радиус по которому будут отобраны соседи точки для вычисления нормали
    @param normals_out [out] сюда будут записаны найденные нормали
*/
template<typename PointType>
void compute_surface_normals (typename pcl::PointCloud<PointType>::Ptr &points, float normal_radius,
                              typename pcl::PointCloud<pcl::Normal>::Ptr &normals_out) {
    pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (normal_radius);
    norm_est.setInputCloud (points);
    norm_est.compute (*normals_out);
}

/**
    @brief Визуализировать облако точек с keypoints
    @param points [in] исходное облако точек
    @param keypoints [in] keypoints
*/
template<typename PointType>
void visualize_keypoints (const typename pcl::PointCloud<PointType>::Ptr points,
                          const typename pcl::PointCloud<PointType>::Ptr keypoints) {
    // Add the points to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points, "points");
    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size (); ++i)
    {
      // Get the point data
      const PointType & p = keypoints->points[i];

      // Pick the radius of the sphere *
      float r = 0.1;
      // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
      //   radius of 2*p.scale is a good illustration of the extent of the keypoint

      // Generate a unique string for each sphere
      std::stringstream ss ("keypoint");
      ss << i;

      // Add a sphere at the keypoint
      viz.addSphere (p, 0.1, 1.0, 0.0, 0.0, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}

/**
    @brief Загрузить облако точек из файла содержащего три столбца X,Y,Z с координатами
    @details Тестовый файл для загрузки должен быть вида
    \code
    -123,32 24,4 123
    155,35 77, 152
    66 88 90
    \endcode
    то есть это последовательность троек координат, используется для загрузки облак точек, получаемых при помощи \ref stereo_match
    @param file [in] имя файла содержащего координаты
    @param cloud [out] заполненное облако точек
    @return Возвращает количество прочитанных строк(троек координат)
*/
int load_from_xyz(const std::string& file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

/**
    @brief Визуализировать два облака точек на небольшом расстоянии друг от друга
    @details Используется для сравнения облака точек. Например до применения фильтра и после
    @param source_points [in] первое облако точек для сравнения
    @param target_points [in] второе облако точек для сравнения
*/
template<typename PointType>
void visualize_2cloud(const typename pcl::PointCloud<PointType>::Ptr source_points,
                      const typename pcl::PointCloud<PointType>::Ptr target_points) {
    typename pcl::PointCloud<PointType>::Ptr copy_source_points (new pcl::PointCloud<PointType>);
    typename pcl::PointCloud<PointType>::Ptr copy_target_points (new pcl::PointCloud<PointType>);

    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (20.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*source_points, *copy_source_points, -translate, no_rotation);

    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*target_points, *copy_target_points, translate, no_rotation);

    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (copy_source_points, "points_source");
    viz.addPointCloud (copy_target_points, "points_target");
    viz.spin ();
}

/**
    @brief Визуализировать два облака точек с левого и правого изображений и найденные correspondences между ними
    @details Используется для просмотра, насколько хорошо найдены correspondences
    @param source_points [in] первое облако точек для сравнения
    @param target_points [in] второе облако точек для сравнения
    @param correspondences [in] correspondences между облаками точек
*/
template<typename PointType>
void visualize_correspondences (const typename pcl::PointCloud<PointType>::Ptr source_points,
                                const typename pcl::PointCloud<PointType>::Ptr target_points,
                                const pcl::Correspondences& correspondences) {
    // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
    // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
    // Create some new point clouds to hold our transformed data
    typename pcl::PointCloud<PointType>::Ptr copy_source_points (new pcl::PointCloud<PointType>);
    typename pcl::PointCloud<PointType>::Ptr copy_target_points (new pcl::PointCloud<PointType>);

    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (20.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*source_points, *copy_source_points, -translate, no_rotation);

    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*target_points, *copy_target_points, translate, no_rotation);

    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (copy_source_points, "points_source");
    viz.addPointCloud (copy_target_points, "points_target");

    // Draw lines between the best corresponding points
    for (size_t i = 0; i < correspondences.size (); ++i)
    {
      // Get the pair of points
      const PointType & p_source = copy_source_points->at( correspondences[i].index_query );
      const PointType & p_target = copy_target_points->at( correspondences[i].index_match );

      // Generate a random (bright) color
      double r = (rand() % 100);
      double g = (rand() % 100);
      double b = (rand() % 100);
      double max_channel = std::max (r, std::max (g, b));
      r /= max_channel;
      g /= max_channel;
      b /= max_channel;

      // Generate a unique string for each line
      std::stringstream ss ("line");
      ss << i;

      // Draw the line
      viz.addLine (p_source, p_target, r, g, b, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}

/**
    @brief Удалить плохие feature(descriptors)
    @param source_points [in] облако точек из левого изображения
    @param source_normal [in] нормали из левого изображения
    @param target_points [in] облако точек из правого изображения
    @param target_normal [in] нормали из правого изображения
    @param original_correspondences [in] найденные correspondences между облаками точек
    @param remaining_correspondence [out] выходные оставшиеся, "хорошие" correspondences между облаками точек
    @param rejector [in] способ фильтрации плохих correspondences. Подробней
*/
template<typename PointType>
void reject_bad_feature(const typename pcl::PointCloud<PointType>::Ptr source_points,
                        const typename pcl::PointCloud<pcl::Normal>::Ptr source_normal,
                        const typename pcl::PointCloud<PointType>::Ptr target_points,
                        const typename pcl::PointCloud<pcl::Normal>::Ptr target_normal,
                        const pcl::Correspondences& original_correspondences,
                        pcl::Correspondences& remaining_correspondence,
                        pcl::registration::CorrespondenceRejector::Ptr rejector) {
    //XXX
    //Приходится конвертировать входящие PointCloud в pcl::PCLPointCloud2
    //поскольку методы базового класса rejector'а ожидают именно pcl::PCLPointCloud2.
    //При этом некоторые реализации rejector'а в своих внутренних методах будут его
    //обратно конвертить в PointCloud, что не рационально и не оптизимированно, но неизбежно если мы хотим использовать
    //интерфейс базового класса.
    pcl::PCLPointCloud2::Ptr msg_source_points(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr msg_source_normal(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr msg_target_points(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr msg_target_normal(new pcl::PCLPointCloud2);

    if(rejector->requiresSourcePoints()) {
        pcl::toPCLPointCloud2 (*source_points, *msg_source_points);
        rejector->setSourcePoints(msg_source_points);
    }

    if(rejector->requiresSourceNormals()) {
        toPCLPointCloud2 (*source_normal, *msg_source_normal);
        rejector->setSourceNormals(msg_source_normal);
    }

    if(rejector->requiresTargetPoints()) {
        toPCLPointCloud2 (*target_points, *msg_target_points);
        rejector->setTargetPoints(msg_target_points);
    }

    if(rejector->requiresTargetNormals()) {
        toPCLPointCloud2 (*target_normal, *msg_target_normal);
        rejector->setTargetNormals(msg_target_normal);
    }
    rejector->getRemainingCorrespondences(original_correspondences, remaining_correspondence);
}

/**
    @brief Отфильтровать облако точек
    @details Используются фильтры StatisticalOutlierRemoval и RadiusOutlierRemoval.
    @attention Эти фильтры принимают на вход дополнительные параметры, они захардкожены в реализации, нужно настраивать их под ваши конкретные нужды в реализации этой функции
    @param cloud [in] облако точек которое "на месте" будет отфильтровано.
*/
template<typename PointType>
void filtered_pointcloud(typename pcl::PointCloud<PointType>::Ptr cloud) {
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (200); //взято от "балды" посмотреть как работает
    sor.setStddevMulThresh (0.1);
    sor.filter (*cloud);

    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(1.8); //взято от "балды" посмотреть как работает
    outrem.setMinNeighborsInRadius (500); //взято от "балды" посмотреть как работает
    outrem.filter (*cloud);
}

/**
    @brief Собрать облака точек в единое облако, выполнив геометрические трансформации между ними
    @details Используется Iterative Closetst Points алгоритм для нахождения матрицы трансформации.
    Первое облако точек принимается как исходное и все остальные облака последовательно на него "наслаиваются".
    Если какое-то из облаков точек не может быть трансформировано(icp.hasConverged()), оно пропускается
    @param clouds [in] вектор облаков точек
    @return Результирующее облако точек
*/
template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr SumClouds(const std::vector<typename pcl::PointCloud<PointType>::ConstPtr>& clouds) {
    typename pcl::PointCloud<PointType>::Ptr final_cloud;
    for(int i = 0; i < clouds.size(); i++) {
        typename pcl::PointCloud<PointType>::Ptr next_cloud = clouds[i];
        if (!final_cloud) {
            final_cloud = next_cloud;
        } else {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setInputCloud(next_cloud);
            icp.setInputTarget(final_cloud);
            typename pcl::PointCloud<PointType>::Ptr transform(new pcl::PointCloud<PointType>);
            icp.align(*transform);
            if (!icp.hasConverged()) {
                PCL_WARN ("Couldn't converged cloud, skip...\n");
                continue;
            }
            *final_cloud += *transform;
        }
    }
    return final_cloud;
}

/**
    @brief Компаратор для std::sort что бы отсортировать последовательность файлов в порядке их создания
    @param p1 [in] путь к первому файлу
    @param p2 [in] путь ко второму файлу
    @return возвращает true если первый файл p1 был создан раньше по времени чем второй p2 и false в противном случае.
*/
bool file_date_comparator (const fs::path &p1, const fs::path &p2);

/**
    @brief Заполнить облако точек на основе файла в формате XY, полученного от программы UrgBenri при работе с лидаром
    @param filename [in] путь к файлу в формате XY
    @param cloud [out] заполненное облако точек
    @return возвращает количество прочитанных троек координат или -1 в случае ошибки при парсинге
*/
int parseUrgBenriXY(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
} //end 3dmap namespace
#endif // UTILS_H

