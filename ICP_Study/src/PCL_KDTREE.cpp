#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/robotics_qi/Data/Kitti/sequences/00/EDGE_PCD/0.pcd", *source_cloud) == -1){
        PCL_ERROR("Cloudn't read file!");
        return -1;
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/robotics_qi/Data/Kitti/sequences/00/EDGE_PCD/1.pcd", *target_cloud) == -1){
        PCL_ERROR("Cloudn't read file!");
        return -1;
    }
    // 依据系统时间创建随机数种子
    // srand(time(NULL));
    // 1. 先用伪随机数来模拟
    int max_point_size = source_cloud->points.size();
    std::cout << "The max point size is: " << max_point_size << std::endl;
    int search_point_index = (rand() % max_point_size);
    std::vector<int> search_point_vector;

    for(int i = 0; i < 300; i++)
        search_point_vector.push_back((rand() % max_point_size));

    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr searchpoint_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 随机筛选出尖锐特征点
    for(int i = 0; i < max_point_size; ++i){
        std::vector<int>::iterator it = std::find(search_point_vector.begin(), search_point_vector.end(), i);
        if(it != search_point_vector.end()){
            searchpoint_cloud->push_back(source_cloud->points[i]);
        }else{
            rest_cloud->push_back(source_cloud->points[i]);
        }
    }

    std::cout << "The search point have: " << searchpoint_cloud->points.size() << std::endl;
    std::cout << "The rest have: " << rest_cloud->points.size() << std::endl;

    // The KD Tree Construction.
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr edge_feature_tree_;
    edge_feature_tree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    edge_feature_tree_->setInputCloud(target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr correspondence_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_point_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempoint_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 随机筛选出尖锐特征点
    int max_point_in_target = target_cloud->points.size();
    for(auto point : searchpoint_cloud->points) {
        edge_feature_tree_->nearestKSearch(point, 5, pointSearchInd, pointSearchSqDis);
        tempoint_cloud->push_back(point);
        for(int i = 0; i < max_point_size; ++i){
            std::vector<int>::iterator it = std::find(pointSearchInd.begin(), pointSearchInd.end(), i);
            if(it != pointSearchInd.end()){
                correspondence_cloud->push_back(target_cloud->points[pointSearchInd[0]]);
            }else{
                rest_point_target->push_back(source_cloud->points[i]);
            }
        }
        pointSearchInd.clear(); pointSearchSqDis.clear();
    }

    std::cout << "correspondence cloud size is: " << correspondence_cloud->points.size() << std::endl;

    // Display the visualiser
    // 初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);  //设置背景颜色为黑色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            rest_color (source_cloud, 255, 255, 255);
    viewer_final->addPointCloud<pcl::PointXYZ> (rest_cloud, rest_color, "rest cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rest cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            searchpoint_color (searchpoint_cloud, 255, 0, 0);


    viewer_final->addPointCloud<pcl::PointXYZ> (tempoint_cloud, searchpoint_color, "searchpoint cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "searchpoint cloud"); //启动可视化

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            correspondence_color(source_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(correspondence_cloud, correspondence_color, "Correspondence Point");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Correspondence Point"); //启动可视化
    viewer_final->addCoordinateSystem (1.0);  //显示XYZ指示轴
    viewer_final->initCameraParameters ();   //初始化摄像头参数
    while (!viewer_final->wasStopped ()){
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (0);
    return 0;
}