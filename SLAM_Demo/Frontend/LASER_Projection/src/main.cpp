//
// Created by Robotics_qi on 2020/7/17.
//

#include <Spherical_View_Projection.h>

int main()
{
    const Configuration config_input{2, -24.8, 64, 1024};
    const std::string path = "/Users/robotics_qi/Data/kitti_dataset/lidar/pcds/000000.pcd";
    std::cout << path << std::endl;
    SphericalConversion conv(config_input);
    conv.loadCloud(path);
    conv.MakeImage();
    auto img = conv.GetImage();
    conv.ShowImg(img);

}