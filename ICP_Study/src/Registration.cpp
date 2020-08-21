//
// Created by Robotics_qi on 2020/8/10.
//

#include <Registration.h>

// Construct
MSP_REGISTRATION_METHOD::MSP_REGISTRATION_METHOD(REGISTRATION_METHOD registration_method) {
    registration_method_ = registration_method;
    iterations_ = 0;
    iter_ = 0;
}

MSP_REGISTRATION_METHOD::~MSP_REGISTRATION_METHOD(){
    if(source_KDTree_pointer != NULL) delete source_KDTree_pointer;
    if(target_KDTree_pointer != NULL) delete target_KDTree_pointer;
}

void MSP_REGISTRATION_METHOD::setIteration(uint32_t iteration){
    iterations_ = iteration;
}

void MSP_REGISTRATION_METHOD::RemoveNANAndINFData(std::vector<Eigen::Vector3d> &point_cloud_input) {
    // 非法数据去除
    for(std::vector<Eigen::Vector3d>::iterator it = point_cloud_input.begin(); it != point_cloud_input.end();){
        Eigen::Vector3d temp_point_cloud = *it;
        if(std::isnan(temp_point_cloud(0)) || std::isnan(temp_point_cloud(1)) || std::isnan(temp_point_cloud(2))
           || std::isinf(temp_point_cloud(0)) || std::isinf(temp_point_cloud(1)) || std::isinf(temp_point_cloud(2)))
            it = point_cloud_input.erase(it);
        else
            it++;
    }
}

void MSP_REGISTRATION_METHOD::setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ>& source_point_cloud_pcl){
    std::vector<Eigen::Vector3d> source_point_cloud_eigen;
    for(int i = 0; i < source_point_cloud_pcl.size(); ++i) {
        std::cout << "Now, the point << " << i << " position is: " << source_point_cloud_pcl[i].x << ", " << source_point_cloud_pcl[i].y << ", " << source_point_cloud_pcl[i].z << std::endl;
        source_point_cloud_eigen.push_back(
                Eigen::Vector3d(source_point_cloud_pcl[i].x, source_point_cloud_pcl[i].y, source_point_cloud_pcl[i].z));
    }
    setSourcePointCloud(source_point_cloud_eigen);
}


void MSP_REGISTRATION_METHOD::setSourcePointCloud(std::vector<Eigen::Vector3d>& source_point_cloud){
    current_source_point_cloud_ = source_point_cloud;
    RemoveNANAndINFData(current_source_point_cloud_);
    std::cout << "Now, we have " << current_source_point_cloud_.size() << " points " << std::endl;
}

void MSP_REGISTRATION_METHOD::setSourcePointCloudNormals(std::vector<Eigen::Vector3d> &normals) {
    current_source_point_cloud_normals = normals;
}

void MSP_REGISTRATION_METHOD::setTargetPointCloudNormals(std::vector<Eigen::Vector3d> &normals) {

}

void MSP_REGISTRATION_METHOD::setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ>& target_point_cloud_pcl){
    std::vector<Eigen::Vector3d> target_point_cloud;
    for(int i = 0; i < target_point_cloud_pcl.size(); ++i)
        target_point_cloud.push_back(Eigen::Vector3d(target_point_cloud_pcl[i].x, target_point_cloud_pcl[i].y, target_point_cloud_pcl[i].z));
    setTargetPointCloud(target_point_cloud);
}

void MSP_REGISTRATION_METHOD::view_source_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_point_cloud_view) {
    pcl::PointXYZ temp_point;
    for (int i = 0; i < current_source_point_cloud_.size(); ++i) {
        temp_point.x = current_source_point_cloud_[i](0);
        temp_point.y = current_source_point_cloud_[i](1);
        temp_point.z = current_source_point_cloud_[i](2);
        source_point_cloud_view->push_back(temp_point);
    }
}

void MSP_REGISTRATION_METHOD::view_target_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_point_cloud_view){
    pcl::PointXYZ temp_point;
    if(current_target_point_cloud_.size() == 0) std::cout << "Haven't read any cloud!" << std::endl;
    for (int i = 0; i < current_target_point_cloud_.size(); ++i) {
        temp_point.x = current_target_point_cloud_[i](0);
        temp_point.y = current_target_point_cloud_[i](1);
        temp_point.z = current_target_point_cloud_[i](2);
        target_point_cloud_view->push_back(temp_point);
    }
}

void MSP_REGISTRATION_METHOD::setTargetPointCloud(std::vector<Eigen::Vector3d>& target_point_cloud_eigen){
    current_target_point_cloud_ = target_point_cloud_eigen;

    if(target_KDTree_pointer != NULL){
//        delete target_KDTree_pointer;
        target_KDTree_pointer = NULL;
    }
    RemoveNANAndINFData(current_target_point_cloud_);

    // 构建KD-Tree
    if(target_KDTree_pointer == NULL){
        target_KDTree_DataBase.resize(3, current_target_point_cloud_.size());
        for(int i = 0; i < current_target_point_cloud_.size(); ++i){
            target_KDTree_DataBase(0, i) = current_target_point_cloud_[i](0);
            target_KDTree_DataBase(1, i) = current_target_point_cloud_[i](1);
            target_KDTree_DataBase(2, i) = current_target_point_cloud_[i](2);
        }
        target_KDTree_pointer = Nabo::NNSearchD::createKDTreeLinearHeap(target_KDTree_DataBase);
    }
    normals_valid_ = false;
}