//
// Created by gatsby on 2019-04-24.
//

#include "ICP.h"

int main(int argc, char **argv)
{
    const std::uint32_t points_size = 20;
    const float points_range = 20;
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(3.141592, Eigen::Vector3d::UnitX());
    Eigen::Vector3d translation = Eigen::Vector3d(1., 2., 3.);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() = translation;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > reference, current;
    std::srand((unsigned)(std::time(NULL)));

    for(int pos = 0; pos < points_size; pos++){
        Eigen::Vector3d point;
        point.x() = (std::rand() % (int)(points_range * 2 * 1000.) - points_range * 1000.) / 1000.;
        point.y() = (std::rand() % (int)(points_range * 2 * 1000.) - points_range * 1000.) / 1000.;
        point.z() = (std::rand() % (int)(points_range * 2 * 1000.) - points_range * 1000.) / 1000.;
        std::cout << "The reference point " << pos << "'s location is " << point.x() << point.y() << point.z() << std::endl;
        reference.push_back(point);
    }

    for(int pos = 0; pos < reference.size(); pos++){
        Eigen::Vector4d point;
        point.x() = reference.at(pos).x();
        point.y() = reference.at(pos).y();
        point.z() = reference.at(pos).z();
        point.w() = 1;

        point = transform * point;
        std::cout << "The current point " << pos << "'s location is " << point.x() << point.y() << point.z() << std::endl;
        current.push_back(Eigen::Vector3d(point.x(), point.y(), point.z()));

        auto estimated = icpPoint2Point(reference, current);
        std::cout << " The true translation is: " << transform.translation().x()
                  << transform.translation().y() << transform.translation().z() << std::endl;
        std::cout << " The estimated translation is: " << estimated.translation().x() << estimated.translation().y()
                  << estimated.translation().z() << std::endl;

    }
}