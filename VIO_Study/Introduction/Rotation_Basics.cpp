//
// Created by Robotics_qi on 2020/4/16.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Matrix3d rotationUpdate(Eigen::Matrix3d &rotation, Eigen::Vector3d &update){
    Eigen::Matrix3d temp_rotation;
    temp_rotation << 0, (-update[2]),  update[1], update[2], 0, (-update[0]), (-update[1]), update[0], 0;
    Eigen::Matrix3d rotation_results = rotation * (Eigen::Matrix3d::Identity() + temp_rotation);
    return rotation_results;
}

Eigen::Matrix3d quaternionUpdate(Eigen::Quaterniond &rotation_quaternion, Eigen::Vector3d &update){
    Eigen::Quaterniond result_quaternion;
    Eigen::Quaterniond temp_rotation_quaternion;
    temp_rotation_quaternion.w() = 1;
    temp_rotation_quaternion.x() = update[0]/2;
    temp_rotation_quaternion.y() = update[1]/2;
    temp_rotation_quaternion.z() = update[2]/2;
    result_quaternion = rotation_quaternion * temp_rotation_quaternion;
    result_quaternion.normalize();
    return result_quaternion.toRotationMatrix();
}

int main()
{
    std::cout << "To Review the study of rotation, we have set a Rotation." << std::endl;
    Eigen::Quaterniond rotation_quaternion(1, 0, 0, 0);
    std::cout << "It's quaternion form is: " << rotation_quaternion.w() << " " << rotation_quaternion.x() << " " << rotation_quaternion.y() << " " << rotation_quaternion.z() << std::endl;
    Eigen::Matrix3d rotationMatrix = rotation_quaternion.toRotationMatrix();
    std::cout << "It's rotation matrix form is: " << std::endl << rotationMatrix << std::endl;
    Eigen::Vector3d rotation_update(0.01, 0.02, 0.03);
    std::cout << "The rotation update value is: " << rotation_update[0] << " " << rotation_update[1] << " " << rotation_update[2] << " " << std::endl;
    Eigen::Matrix3d rotation_updated = rotationUpdate(rotationMatrix, rotation_update);
    Eigen::Matrix3d quaternion_updated = quaternionUpdate(rotation_quaternion, rotation_update);
    std::cout << "The updated value from rotation in rotation matrix is: " << std::endl << rotation_updated << std::endl;
    std::cout << "The updated value from quaternion in rotation matrix is: " << std::endl << quaternion_updated << std::endl;
}