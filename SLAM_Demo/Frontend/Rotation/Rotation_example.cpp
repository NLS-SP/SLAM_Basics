#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char **argv)
{
    Eigen::Quaterniond q1(1,0,0,0);  //定义两个四元数
    Eigen::Quaterniond q2(1,0,0,0);
    q1.normalize();  //归一化
    q2.normalize();
    Eigen::Vector3d t1(0,0,0);  //定义两个平移向量
    Eigen::Vector3d t2(1,0,0);

    //Eigen::Matrix3d R1=q1.toRotationMatrix();
    //Eigen::Matrix3d R2=q2.toRotationMatrix();

    Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();  //定义两个变换矩阵
    T1.rotate(q1);  //由四元数确定旋转(也可由旋转矩阵、旋转向量确定旋转)
    T1.pretranslate(t1);

    Eigen::Isometry3d T2=Eigen::Isometry3d::Identity();
    T2.rotate(q2);
    T2.pretranslate(t2);

    Eigen::Vector3d v1(0,0,0);  //小萝卜一号在自身坐标系下看到的某个点坐标
    Eigen::Vector3d p_world=T1.inverse()*v1;  //根据变换关系求出该点坐标的世界坐标
    Eigen::Vector3d P_C2=T2*p_world;  //由世界坐标反解出其在小萝卜二号自身坐标系下看到的该点坐标

    cout<<"坐标为:"<<P_C2.transpose()<<endl;
    return 0;
}