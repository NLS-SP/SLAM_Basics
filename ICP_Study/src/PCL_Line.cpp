// MLStoALS.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/registration/icp.h>
#include<pcl/visualization/pcl_visualizer.h>//可视化头文件
#include <pcl/filters/radius_outlier_removal.h>
#include<pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "lasreader.hpp"
#include <opencv2/opencv_modules.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#define MATCHES_DIST_THRESHOLD 25
#define eps 0.00000000001


//修改1019
using namespace std;
using namespace cv;
using namespace cv::line_descriptor;
static std::string convertFilePath(const std::string& file)
{
    int i = 0;
    std::string s(file);
    for (i = 0; i < s.size(); ++i)
    {
        if (s[i] == '/')
            s[i] = '\\';
    }
    return s;
}
typedef struct {
    int grayScale = 0;
    float maxdif_height = 0;
    float region_difheight = 0;
    float max_height = 0;
    int candidate = 0;
    vector<int> indexID;

}flat_grid;
typedef struct {
    vector<int> als_index;
    vector<Vec2f> xy_drift;
}lines_combination;
typedef struct {
    pcl::PointXYZ startpoint;
    pcl::PointXYZ endpoint;
}Point_vec;
int findmaxValue(Mat img, int rol, int col)
{
    int maxV = 0;
    for (int i = rol - 1; i <= rol + 1; i++)
    {
        for (int j = col - 1; j <= col + 1; j++)
        {
            if (i < 0 || j < 0) continue;
            if (maxV < img.ptr<uchar>(i)[j]) maxV = img.ptr<uchar>(i)[j];
        }
    }
    return maxV;
}
void thinImage(Mat & srcImg) {
    vector<Point> deleteList;
    int neighbourhood[9];
    int nl = srcImg.rows;
    int nc = srcImg.cols;
    bool inOddIterations = true;
    while (true) {
        for (int j = 1; j < (nl - 1); j++) {
            uchar* data_last = srcImg.ptr<uchar>(j - 1);
            uchar* data = srcImg.ptr<uchar>(j);
            uchar* data_next = srcImg.ptr<uchar>(j + 1);
            for (int i = 1; i < (nc - 1); i++) {
                if (data[i] == 255) {
                    int whitePointCount = 0;
                    neighbourhood[0] = 1;
                    if (data_last[i] == 255) neighbourhood[1] = 1;
                    else  neighbourhood[1] = 0;
                    if (data_last[i + 1] == 255) neighbourhood[2] = 1;
                    else  neighbourhood[2] = 0;
                    if (data[i + 1] == 255) neighbourhood[3] = 1;
                    else  neighbourhood[3] = 0;
                    if (data_next[i + 1] == 255) neighbourhood[4] = 1;
                    else  neighbourhood[4] = 0;
                    if (data_next[i] == 255) neighbourhood[5] = 1;
                    else  neighbourhood[5] = 0;
                    if (data_next[i - 1] == 255) neighbourhood[6] = 1;
                    else  neighbourhood[6] = 0;
                    if (data[i - 1] == 255) neighbourhood[7] = 1;
                    else  neighbourhood[7] = 0;
                    if (data_last[i - 1] == 255) neighbourhood[8] = 1;
                    else  neighbourhood[8] = 0;
                    for (int k = 1; k < 9; k++) {
                        whitePointCount += neighbourhood[k];
                    }
                    if ((whitePointCount >= 2) && (whitePointCount <= 6)) {
                        int ap = 0;
                        if ((neighbourhood[1] == 0) && (neighbourhood[2] == 1)) ap++;
                        if ((neighbourhood[2] == 0) && (neighbourhood[3] == 1)) ap++;
                        if ((neighbourhood[3] == 0) && (neighbourhood[4] == 1)) ap++;
                        if ((neighbourhood[4] == 0) && (neighbourhood[5] == 1)) ap++;
                        if ((neighbourhood[5] == 0) && (neighbourhood[6] == 1)) ap++;
                        if ((neighbourhood[6] == 0) && (neighbourhood[7] == 1)) ap++;
                        if ((neighbourhood[7] == 0) && (neighbourhood[8] == 1)) ap++;
                        if ((neighbourhood[8] == 0) && (neighbourhood[1] == 1)) ap++;
                        if (ap == 1) {
                            if (inOddIterations && (neighbourhood[3] * neighbourhood[5] * neighbourhood[7] == 0)
                                && (neighbourhood[1] * neighbourhood[3] * neighbourhood[5] == 0)) {
                                deleteList.push_back(Point(i, j));
                            }
                            else if (!inOddIterations && (neighbourhood[1] * neighbourhood[5] * neighbourhood[7] == 0)
                                     && (neighbourhood[1] * neighbourhood[3] * neighbourhood[7] == 0)) {
                                deleteList.push_back(Point(i, j));
                            }
                        }
                    }
                }
            }
        }
        if (deleteList.size() == 0)
            break;
        for (size_t i = 0; i < deleteList.size(); i++) {
            Point tem;
            tem = deleteList[i];
            uchar* data = srcImg.ptr<uchar>(tem.y);
            data[tem.x] = 0;
        }
        deleteList.clear();

        inOddIterations = !inOddIterations;
    }
}

int main()
{

    /**************     车载点云处理         ****************/

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointXYZI>);
    ///读取las文件///////
    const char* your_MLS_file_path = "1.las";
    //laslib只允许'\\'格式的文件路径。
    std::string lasFile = convertFilePath(your_MLS_file_path);

    //打开las文件
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(lasFile.c_str());
    LASreader* lasreader = lasreadopener.open();
    size_t count = lasreader->header.number_of_point_records;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mls_p(new pcl::PointCloud<pcl::PointXYZI>);

    mls_p->resize(count);
    mls_p->width = count;
    mls_p->height = 1;
    mls_p->is_dense = false;
    size_t i = 0;
    while (lasreader->read_point() && i < count)
    {
        mls_p->points[i].x = lasreader->point.get_x();
        mls_p->points[i].y = lasreader->point.get_y();
        mls_p->points[i].z = lasreader->point.get_z();
        mls_p->points[i].intensity = lasreader->point.get_intensity();
        ++i;
    }
    cout << "读取las点云数量:" << i << endl;
    //////////////滤波//////
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;  //创建滤波器

    outrem.setInputCloud(mls_p);    //设置输入点云
    outrem.setRadiusSearch(1);     //设置半径为0.5的范围内找临近点
    outrem.setMinNeighborsInRadius(5); //设置查询点的邻域点集数小于10的删除
    // apply filter
    outrem.filter(*cloud_mls);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
    std::cerr << "Cloud after filtering" << endl;
    std::cerr << cloud_mls->size() << endl;
    cloud_mls->width = cloud_mls->points.size();
    cloud_mls->height = 1;
    cloud_mls->is_dense = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr Mcloud_flitered(new pcl::PointCloud<pcl::PointXYZI>);
    vector<int>pointIndices;
    //提取点云最值
    pcl::PointXYZI min_mls;
    pcl::PointXYZI max_mls;
    pcl::getMinMax3D(*cloud_mls, min_mls, max_mls);
    //输入格网间隔
    float Mgrid_distance;
    cerr << "输入MLS格网间隔值:" << endl;
    cin >> Mgrid_distance;

    float threshold_heightDiff;
    cerr << "输入MLS区域高差阈值" << endl;
    cin >> threshold_heightDiff;

    //计算区域内格网XY方向数量
    cerr << "X方向最大值:" << max_mls.x << endl;
    cerr << "X方向最小值:" << min_mls.x << endl;
    int width = int((max_mls.x - min_mls.x) / Mgrid_distance) + 1;

    cerr << "Y方向最大值:" << max_mls.y << endl;
    cerr << "Y方向最小值:" << min_mls.y << endl;
    int height = int((max_mls.y - min_mls.y) / Mgrid_distance) + 1;
    cerr << width << endl << height << endl;

    //构建二维平面格网
    flat_grid **voxel = new flat_grid*[width];
    for (int i = 0; i < width; ++i)
        voxel[i] = new flat_grid[height];
    int row, col;
    for (size_t i = 0; i < cloud_mls->points.size(); i++)
    {
        row = int((cloud_mls->points[i].x - min_mls.x) / Mgrid_distance);
        col = int((cloud_mls->points[i].y - min_mls.y) / Mgrid_distance);
        voxel[row][col].indexID.push_back(i);
        if (voxel[row][col].grayScale != 5)
            voxel[row][col].grayScale++;
    }
    int count_candidate = 0;
    for (int i = 0; i < width; i++)   //遍历格网
    {
        for (int j = 0; j < height; j++)
        {
            if (voxel[i][j].grayScale >= 1)
            {
                float  max_h = 0.0;
                for (int num = 0; num < voxel[i][j].indexID.size(); num++)
                {
                    if (cloud_mls->points[voxel[i][j].indexID[num]].z > max_h)max_h = cloud_mls->points[voxel[i][j].indexID[num]].z;
                }
                voxel[i][j].region_difheight = max_h - min_mls.z;
            }
            if (voxel[i][j].grayScale == 5)         //提取非空格网数
            {
                int point_num = 0;
                float  max_h = 0.0;
                for (int num = 0; num < voxel[i][j].indexID.size(); num++)
                {
                    if (cloud_mls->points[voxel[i][j].indexID[num]].z > max_h)max_h = cloud_mls->points[voxel[i][j].indexID[num]].z;
                }

                if ((max_h - min_mls.z) > threshold_heightDiff)
                {
                    voxel[i][j].candidate = 1;
                    count_candidate++;
                }
            }
        }
    }
    cerr << "总格网数：" << width * height << endl;
    cerr << "候选格网数：" << count_candidate << endl;
    int num_continues = 0;
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (voxel[i][j].candidate == 1)
            {
                num_continues++;
                pcl::PointCloud<pcl::PointXYZI>::Ptr voxelPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);   //构建格网点云集
                voxelPointCloudPtr->width = voxel[i][j].indexID.size();
                voxelPointCloudPtr->height = 1;
                voxelPointCloudPtr->is_dense = false;
                voxelPointCloudPtr->resize(voxelPointCloudPtr->width * voxelPointCloudPtr->height);
                for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //读取格网点云数据
                {

                    voxelPointCloudPtr->points[k].x = cloud_mls->points[voxel[i][j].indexID[k]].x;
                    voxelPointCloudPtr->points[k].y = cloud_mls->points[voxel[i][j].indexID[k]].y;
                    voxelPointCloudPtr->points[k].z = cloud_mls->points[voxel[i][j].indexID[k]].z;
                }
                pcl::PointXYZI min_voxel;
                pcl::PointXYZI max_voxel;
                pcl::getMinMax3D(*voxelPointCloudPtr, min_voxel, max_voxel);
                for (int num = 0; num < voxelPointCloudPtr->points.size(); num++)
                {
                    if (fabs(voxelPointCloudPtr->points[num].z - max_voxel.z) < 30)
                    {
                        pointIndices.push_back(voxel[i][j].indexID[num]);
                    }
                }
            }
        }
    }
    cerr << "选取格网数：" << num_continues << endl;

    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(pointIndices);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // Extract the inliers
    extract.setInputCloud(cloud_mls);
    extract.setIndices(index_ptr);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract.filter(*Mcloud_flitered);
    cerr << "输出点云数量:" << Mcloud_flitered->size() << endl;

    pcl::visualization::PCLVisualizer viewer("视窗");
    viewer.setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            target_color(cloud_mls, 205, 92, 92);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            src_color(Mcloud_flitered, 144, 238, 238);


    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer.addPointCloud<pcl::PointXYZI>(cloud_mls, target_color, "sample", v1);//显示点云，其中fildColor为颜色显示


    viewer.addPointCloud<pcl::PointXYZI>(Mcloud_flitered, src_color, "trans", v2);//显示点云，其中fildColor为颜色显示


    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");//设置点云大小


    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "trans");//设置点云大小

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    //////////栅格化///////////
    //构建二维平面格网
    flat_grid **voxel_1 = new flat_grid*[width];
    for (int i = 0; i < width; ++i)
        voxel_1[i] = new flat_grid[height];
    for (size_t i = 0; i < Mcloud_flitered->points.size(); i++)
    {
        row = int((Mcloud_flitered->points[i].x - min_mls.x) / Mgrid_distance);
        col = int((Mcloud_flitered->points[i].y - min_mls.y) / Mgrid_distance);
        voxel_1[row][col].indexID.push_back(i);
        if (voxel_1[row][col].grayScale < 1)
            voxel_1[row][col].grayScale++;
    }
    float max_heightDiff = 0;
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (voxel_1[i][j].grayScale == 1)         //提取非空格网数
            {
                float max_height = -999.0;
                //float min_height = 1000.0;
                for (int num = 0; num < voxel_1[i][j].indexID.size(); num++)
                {
                    if (Mcloud_flitered->points[voxel_1[i][j].indexID[num]].z > max_height) max_height = Mcloud_flitered->points[voxel_1[i][j].indexID[num]].z;
                    //if (cloud_flitered->points[voxel_1[i][j].indexID[num]].z < min_height) min_height = cloud_flitered->points[voxel_1[i][j].indexID[num]].z;
                }
                // 提取格网最大高程值作为灰度（车载）
                voxel_1[i][j].max_height = max_height;
                // 提取格网高差作为灰度（机载）
                voxel_1[i][j].region_difheight = max_height - min_mls.z;
                if (voxel_1[i][j].region_difheight > max_heightDiff) max_heightDiff = voxel_1[i][j].region_difheight;
            }
        }
    }
    // 机载点云灰度转换公式
    float scale_trans = 255.0 / max_heightDiff;
    //写入Mat图像
    Mat image(width, height, CV_8UC1, Scalar(0));
    uchar a1 = 0;
    for (int i = 0; i < width; i++)  //image.at<uchar>(i,j)
    {
        uchar* data = image.ptr<uchar>(i);
        for (int j = 0; j < height; j++)
        {
            if (voxel_1[i][j].grayScale != 0)
            {
                data[j] = uchar((voxel_1[i][j].region_difheight) * scale_trans);
                a1 = data[j] > a1 ? data[j] : a1;
            }
        }
    }

    //namedWindow("图像", CV_WINDOW_NORMAL);
    //imshow("图像", image);

    ////////骨架提取算法////////////
    Mat binaryImage(width, height, CV_8UC1);
    threshold(image, binaryImage, 20, 255, CV_THRESH_BINARY);
    imshow("binary", binaryImage);
    thinImage(binaryImage);
    namedWindow("output", CV_WINDOW_AUTOSIZE);
    imshow("output", binaryImage);
    //////霍夫直线检测//////////
    vector<Vec4f> line_data;
    HoughLinesP(binaryImage, line_data, 1, CV_PI / 180.0, 10, 10, 2);  //输入图像，输出的极坐标来表示直线，像素扫描步长，极坐标角度步长，判断直线点数的阈值，最小直线长度，线段上最近两点之间的阈值
    vector<Vec4f> lines_2;
    ///////因为通常边界会画出两条线，舍弃端点值没有灰度的线
    for (size_t i = 0; i < line_data.size(); i++) {
        Vec4f temp = line_data[i];
        int a = findmaxValue(image, int(temp[3]), int(temp[2]));
        int b = findmaxValue(image, int(temp[1]), int(temp[0]));
        //int a = image.at<uchar>(int(temp[3]), int(temp[2]));
        //int b = image.at<uchar>(int(temp[1]), int(temp[0]));
        if (a != 0 && b != 0 )//&& fabs((a - b) / scale_trans) <= 10)
        {
            lines_2.push_back(temp);
        }

    }
    vector<Point_vec>mls_xyzline(lines_2.size());
    for (size_t i = 0; i < lines_2.size(); i++)
    {
        Vec4f temp = lines_2[i];
        int a = findmaxValue(image, int(temp[3]), int(temp[2]));
        int b = findmaxValue(image, int(temp[1]), int(temp[0]));
        float x_1 = int(temp[3]) * Mgrid_distance + min_mls.x;
        float y_1 = int(temp[2]) * Mgrid_distance + min_mls.y;
        float z_1 = a / scale_trans + min_mls.z;
        float x_2 = int(temp[1]) * Mgrid_distance + min_mls.x;
        float y_2 = int(temp[0]) * Mgrid_distance + min_mls.y;
        float z_2 = b / scale_trans + min_mls.z;
        float z = z_1 > z_2 ? z_1 : z_2;
        if (x_1 < x_2)
        {
            mls_xyzline[i].startpoint = pcl::PointXYZ(x_1, y_1, z);
            mls_xyzline[i].endpoint = pcl::PointXYZ(x_2, y_2, z);
        }
        else
        {
            mls_xyzline[i].startpoint = pcl::PointXYZ(x_2, y_2, z);
            mls_xyzline[i].endpoint = pcl::PointXYZ(x_1, y_1, z);
        }
    }
    //ls->drawSegments(image_1, lines_1);
    //namedWindow("LSD", CV_WINDOW_AUTOSIZE);
    //imshow("LSD", image_1);
    Scalar color = Scalar(255);
    //写入DSM影像
    Mat MdsmImg(width, height, CV_8UC1, Scalar(0));
    for (int i = 0; i < width; i++)  //image.at<uchar>(i,j)
    {
        uchar* data = MdsmImg.ptr<uchar>(i);
        for (int j = 0; j < height; j++)
        {
            data[j] = uchar((voxel[i][j].region_difheight) * 255.0 / (max_mls.z - min_mls.z));
        }
    }
    imwrite("1.png", MdsmImg);
    Mat MdsmImg_1(MdsmImg);
    cvtColor(MdsmImg_1, MdsmImg, COLOR_GRAY2RGB);
    for (size_t i = 0; i < line_data.size(); i++) {
        Vec4f temp = line_data[i];
        //line(image, Point(temp[0], temp[1]), Point(temp[2], temp[3]), color, 1);
        line(MdsmImg, Point(temp[0], temp[1]), Point(temp[2], temp[3]), Scalar(0, 0, 255), 1);
    }

    namedWindow("Mdsm", CV_WINDOW_AUTOSIZE);
    imshow("Mdsm", MdsmImg);
    waitKey(100);
    for (int i = 0; i < width; ++i)
        delete[] voxel_1[i];
    delete[] voxel_1;
    for (int i = 0; i < width; ++i)
        delete[] voxel[i];
    delete[] voxel;
    /****************机载点云处理*******************/
    ///读取las文件///////
    const char* your_ALS_file_path = "2.las";
    //laslib只允许'\\'格式的文件路径。
    std::string lasFile_1 = convertFilePath(your_ALS_file_path);
    //打开las文件
    LASreadOpener lasreadopener_1;
    lasreadopener_1.set_file_name(lasFile_1.c_str());
    LASreader* lasreader_A = lasreadopener_1.open();
    size_t count_1 = lasreader_A->header.number_of_point_records;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Als_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    Als_cloud->resize(count_1);
    Als_cloud->width = count_1;
    Als_cloud->height = 1;
    Als_cloud->is_dense = false;
    size_t i_1 = 0;
    while (lasreader_A->read_point() && i_1 < count_1)
    {
        Als_cloud->points[i_1].x = lasreader_A->point.get_x();
        Als_cloud->points[i_1].y = lasreader_A->point.get_y();
        Als_cloud->points[i_1].z = lasreader_A->point.get_z();
        Als_cloud->points[i_1].intensity = lasreader_A->point.get_intensity();
        ++i_1;
    }
    cout << "读取ALS点云数量:" << i_1 << endl;
    ///提取格网内部最高点云///////
    //提取点云最值
    pcl::PointXYZI min_als;
    pcl::PointXYZI max_als;
    pcl::getMinMax3D(*Als_cloud, min_als, max_als);
    //输入格网间隔
    float Agrid_distance;
    cerr << "输入ALS格网间隔值:" << endl;
    cin >> Agrid_distance;

    //计算区域内格网XYZ方向数量
    cerr << "X方向最大值:" << max_als.x << endl;
    cerr << "X方向最小值:" << min_als.x << endl;
    int width_als = int((max_als.x - min_als.x) / Agrid_distance) + 1;

    cerr << "Y方向最大值:" << max_als.y << endl;
    cerr << "Y方向最小值:" << min_als.y << endl;
    int height_als = int((max_als.y - min_als.y) / Agrid_distance) + 1;

    cerr << "区域最大高差:" << max_als.z - min_als.z << endl;

    //构建二维平面格网
    flat_grid **voxel_2 = new flat_grid*[width_als];
    for (int i = 0; i < width_als; ++i)
        voxel_2[i] = new flat_grid[height_als];
    int row_als, col_als;
    for (size_t i = 0; i < Als_cloud->points.size(); i++)
    {
        row_als = int((Als_cloud->points[i].x - min_als.x) / Agrid_distance);
        col_als = int((Als_cloud->points[i].y - min_als.y) / Agrid_distance);
        voxel_2[row_als][col_als].indexID.push_back(i);
        if (voxel_2[row_als][col_als].grayScale < 1)
            voxel_2[row_als][col_als].grayScale++;
    }
    cerr << "格网数量：" << width_als * height_als << endl;
    int count_grid = 0;
    vector<int>pointIndices_als;
    float roofHeight = 0;
    cerr << "输入屋顶高程:" << endl;
    cin >> roofHeight;
    //提取屋顶边沿点云
    for (int i = 0; i < width_als; i++)
    {
        for (int j = 0; j < height_als; j++)
        {
            if (voxel_2[i][j].grayScale == 1)
            {
                count_grid++;
                pcl::PointCloud<pcl::PointXYZI>::Ptr voxelPointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);   //构建格网点云集
                voxelPointCloudPtr->width = voxel_2[i][j].indexID.size();
                voxelPointCloudPtr->height = 1;
                voxelPointCloudPtr->is_dense = false;
                voxelPointCloudPtr->resize(voxelPointCloudPtr->width * voxelPointCloudPtr->height);
                for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //读取格网点云数据
                {

                    voxelPointCloudPtr->points[k].x = Als_cloud->points[voxel_2[i][j].indexID[k]].x;
                    voxelPointCloudPtr->points[k].y = Als_cloud->points[voxel_2[i][j].indexID[k]].y;
                    voxelPointCloudPtr->points[k].z = Als_cloud->points[voxel_2[i][j].indexID[k]].z;
                }
                pcl::PointXYZI min;
                pcl::PointXYZI max;
                pcl::getMinMax3D(*voxelPointCloudPtr, min, max);
                voxel_2[i][j].region_difheight = max.z - min_als.z;
                for (size_t k = 0; k < voxelPointCloudPtr->points.size(); k++)     //读取格网点云数据
                {
                    if (voxelPointCloudPtr->points[k].z - min_als.z >= roofHeight) pointIndices_als.push_back(voxel_2[i][j].indexID[k]);;
                }
            }
        }
    }
    cout << "输出格网数：" << count_grid << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Acloud_flitered(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<std::vector<int>> index_Aptr = boost::make_shared<std::vector<int>>(pointIndices_als);
    pcl::ExtractIndices<pcl::PointXYZI> Aextract;
    // Extract the inliers
    Aextract.setInputCloud(Als_cloud);
    Aextract.setIndices(index_Aptr);
    Aextract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    Aextract.filter(*Acloud_flitered);
    cerr << "输出点云数量:" << Acloud_flitered->size() << endl;
    // 进行半径滤波
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;   //创建滤波器对象
    sor.setInputCloud(Acloud_flitered);                           //设置待滤波的点云
    sor.setRadiusSearch(4);                               //设置在进行统计时考虑查询点临近点数
    sor.setMinNeighborsInRadius(20); //设置查询点的邻域点集数小于2的删除
    sor.setKeepOrganized(false);  //如果设置为true,原文件的滤除点会被置为nan
    sor.filter(*cloud_1);                    //存储
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_1->size() << endl;

    pcl::visualization::PCLVisualizer viewer_1("视窗1");
    viewer_1.setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            target_Acolor(cloud_1, 205, 92, 92);

    viewer_1.addPointCloud<pcl::PointXYZI>(cloud_1, target_Acolor, "1");//显示点云，其中fildColor为颜色显示

    viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "1");//设置点云大小
    while (!viewer_1.wasStopped())
    {
        viewer_1.spinOnce();
    }
    //////////栅格化///////////
    pcl::PointXYZI min_p;  //用于存放三个轴最小值
    pcl::PointXYZI max_p;
    pcl::getMinMax3D(*cloud_1, min_p, max_p);
    //构建二维平面格网
    flat_grid **voxel_3 = new flat_grid*[width_als];
    for (int i = 0; i < width_als; ++i)
        voxel_3[i] = new flat_grid[height_als];
    for (size_t i = 0; i < cloud_1->points.size(); i++)
    {
        float a = cloud_1->points[i].x;
        float b = cloud_1->points[i].y;
        row_als = int((cloud_1->points[i].x - min_als.x) / Agrid_distance);
        col_als = int((cloud_1->points[i].y - min_als.y) / Agrid_distance);
        voxel_3[row_als][col_als].indexID.push_back(i);
        if (voxel_3[row_als][col_als].grayScale < 1)
            voxel_3[row_als][col_als].grayScale++;
    }
    for (int i = 0; i < width_als; i++)
    {
        for (int j = 0; j < height_als; j++)
        {
            if (voxel_3[i][j].grayScale == 1)         //提取非空格网数
            {
                float max_height = -999.0;
                for (int num = 0; num < voxel_3[i][j].indexID.size(); num++)
                {
                    if (cloud_1->points[voxel_3[i][j].indexID[num]].z > max_height) max_height = cloud_1->points[voxel_3[i][j].indexID[num]].z;
                }
                // 提取格网最大高程值作为灰度（车载）
                voxel_3[i][j].max_height = max_height;
                // 提取格网高差作为灰度（机载）
                voxel_3[i][j].region_difheight = max_height - min_als.z;
            }
        }
    }
    // 机载点云灰度转换公式 float scale_trans = 255.0 / max_heightDiff;
    // 车载点云灰度转换公式
    float scale_Atrans = 255.0 / (max_als.z - min_als.z);
    //写入Mat图像
    Mat image_1(width_als, height_als, CV_8UC1, Scalar(0));
    for (int i = 0; i < width_als; i++)  //image.at<uchar>(i,j)
    {
        uchar* data = image_1.ptr<uchar>(i);
        for (int j = 0; j < height_als; j++)
        {
            if (voxel_3[i][j].grayScale != 0)
            {
                // 车载点云灰度赋值
                data[j] = uchar(voxel_3[i][j].region_difheight * scale_Atrans);
                // 机载点云灰度赋值 data[j] = uchar((voxel_1[i][j].max_heightDiff) * scale_trans);
            }
        }
    }

    // Create and LSD detector with standard or no refinement.
#if 1
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
#else
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
#endif

    double start = double(getTickCount());
    vector<Vec4f> lines_std;
    vector<Vec4f> lines_1;
    ls->detect(image_1, lines_std);
    //写入DSM影像
    Mat AdsmImg(width_als, height_als, CV_8UC1, Scalar(0));
    for (int i = 0; i < width_als; i++)  //image.at<uchar>(i,j)
    {
        uchar* data = AdsmImg.ptr<uchar>(i);
        for (int j = 0; j < height_als; j++)
        {
            if (voxel_2[i][j].grayScale != 0)
            {
                // 车载点云灰度赋值
                data[j] = uchar(voxel_2[i][j].region_difheight * scale_Atrans);
            }
        }
    }
    imwrite("2.png", AdsmImg);
    ///////因为通常边界会画出两条线，舍弃端点值没有灰度的线
    for (size_t i = 0; i < lines_std.size(); i++) {
        Vec4f temp = lines_std[i];
        int a = findmaxValue(image_1, int(temp[3]), int(temp[2]));
        int b = findmaxValue(image_1, int(temp[1]), int(temp[0]));
        //int a = image.at<uchar>(int(temp[3]), int(temp[2]));
        //int b = image.at<uchar>(int(temp[1]), int(temp[0]));
        if (a != 0 || b != 0) //&& fabs((a - b) / scale_Atrans) <= 10)
        {
            lines_1.push_back(temp);
        }

    }
    vector<Point_vec> als_xyzline(lines_1.size());
    for (size_t i = 0; i < lines_1.size(); i++)
    {
        Vec4f temp = lines_1[i];
        int a = findmaxValue(image_1, int(temp[3]), int(temp[2]));
        int b = findmaxValue(image_1, int(temp[1]), int(temp[0]));
        float x_1 = int(temp[3]) * Agrid_distance + min_als.x;
        float y_1 = int(temp[2]) * Agrid_distance + min_als.y;
        float z_1 = a / scale_Atrans + min_als.z;
        float x_2 = int(temp[1]) * Agrid_distance + min_als.x;
        float y_2 = int(temp[0]) * Agrid_distance + min_als.y;
        float z_2 = b / scale_Atrans + min_als.z;
        float z = z_1 > z_2 ? z_1 : z_2;
        if (x_1 < x_2)
        {
            als_xyzline[i].startpoint = pcl::PointXYZ(x_1, y_1, z);
            als_xyzline[i].endpoint = pcl::PointXYZ(x_2, y_2, z);
        }
        else
        {
            als_xyzline[i].startpoint = pcl::PointXYZ(x_2, y_2, z);
            als_xyzline[i].endpoint = pcl::PointXYZ(x_1, y_1, z);
        }
    }
    //ls->drawSegments(image_1, lines_1);
    //namedWindow("LSD", CV_WINDOW_AUTOSIZE);
    //imshow("LSD", image_1);
    // Show found lines
    Mat AdsmImg_1(AdsmImg);
    ls->drawSegments(AdsmImg_1, lines_1);
    namedWindow("Adsm", CV_WINDOW_AUTOSIZE);
    imshow("Adsm", AdsmImg_1);
    for (int i = 0; i < width_als; ++i)
        delete[] voxel_2[i];
    delete[] voxel_2;
    for (int i = 0; i < width_als; ++i)
        delete[] voxel_3[i];
    delete[] voxel_3;


    /*******线特征匹配***********/
    vector<Vec6f>mls_lines, als_lines;
    for (size_t i = 0; i < lines_2.size(); i++)    //lines_2代表提出的车载特征线
    {
        Vec4f temp = lines_2[i];
        if (temp[1] > temp[3])
        {
            float x_start = int(temp[3]) * Mgrid_distance + min_mls.x;
            float y_start = int(temp[2]) * Mgrid_distance + min_mls.y;
            float x_end = int(temp[1]) * Mgrid_distance + min_mls.x;
            float y_end = int(temp[0]) * Mgrid_distance + min_mls.y;
            float angle = atan((y_end - y_start) / (x_end - x_start)) / 3.14 * 180.0;
            float lineLength = sqrt((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));
            Vec6f keyline(x_start, y_start, x_end, y_end, angle,lineLength);
            mls_lines.push_back(keyline);
        }
        else
        {
            float x_start = int(temp[1]) * Mgrid_distance + min_mls.x;
            float y_start = int(temp[0]) * Mgrid_distance + min_mls.y;
            float x_end = int(temp[3]) * Mgrid_distance + min_mls.x;
            float y_end = int(temp[2]) * Mgrid_distance + min_mls.y;
            float angle = atan((y_end - y_start) / (x_end - x_start)) / 3.14 * 180.0;
            float lineLength = sqrt((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));
            Vec6f keyline(x_start, y_start, x_end, y_end, angle, lineLength);
            mls_lines.push_back(keyline);
        }
    }
    for (size_t i = 0; i < lines_1.size(); i++)   //lines_1代表提取出的机载特征线
    {
        Vec4f temp = lines_1[i];
        if (temp[1] > temp[3])
        {
            float x_start = int(temp[3]) * Agrid_distance + min_als.x;
            float y_start = int(temp[2]) * Agrid_distance + min_als.y;
            float x_end = int(temp[1]) * Agrid_distance + min_als.x;
            float y_end = int(temp[0]) * Agrid_distance + min_als.y;
            float angle = atan((y_end - y_start) / (x_end - x_start)) / 3.14 * 180.0;
            float lineLength = sqrt((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));
            Vec6f keyline(x_start, y_start, x_end, y_end, angle, lineLength);
            als_lines.push_back(keyline);
        }
        else
        {
            float x_start = int(temp[1]) * Agrid_distance + min_als.x;
            float y_start = int(temp[0]) * Agrid_distance + min_als.y;
            float x_end = int(temp[3]) * Agrid_distance + min_als.x;
            float y_end = int(temp[2]) * Agrid_distance + min_als.y;
            float angle = atan((y_end - y_start) / (x_end - x_start)) / 3.14 * 180.0;
            float lineLength = sqrt((x_end - x_start) * (x_end - x_start) + (y_end - y_start) * (y_end - y_start));
            Vec6f keyline(x_start, y_start, x_end, y_end, angle, lineLength);
            als_lines.push_back(keyline);
        }
    }
    ///以车载点云特征线为基础寻找同名线对
    vector<lines_combination> Mmatch(mls_lines.size());
    for (size_t i = 0; i < mls_lines.size(); i++)
    {
        float mls_angle = mls_lines[i][4];
        float mls_length = mls_lines[i][5];
        for (int j = 0; j < als_lines.size(); j++)
        {
            if (abs(als_lines[j][4] - mls_angle) < 5)
            {
                float als_length = als_lines[j][5];
                if ((mls_length / als_length) > 0.67 && (mls_length / als_length) < 1.5 && fabs(mls_lines[i][0] - als_lines[j][0]) < 100 && fabs(mls_lines[i][1] - als_lines[j][1]) < 100)
                {
                    Vec2f temp;
                    temp[0] = mls_lines[i][0] - als_lines[j][0];
                    temp[1] = mls_lines[i][1] - als_lines[j][1];

                    Mmatch[i].als_index.push_back(j);
                    Mmatch[i].xy_drift.push_back(temp);
                }
            }
        }

    }
    ////寻找最佳的匹配组合
    vector <int> vaild_index;
    double num_match = 1;
    for (int i = 0; i < mls_lines.size(); i++)
    {
        if (Mmatch[i].als_index.size() != 0)
        {
            num_match *= Mmatch[i].als_index.size();
            vaild_index.push_back(i);
        }
    }

    //构建一个有对应情况的存储数组
    vector<lines_combination> Mmatch1(vaild_index.size());
    for (int i = 0; i < vaild_index.size(); i++)
    {
        Mmatch1[i] = Mmatch[vaild_index[i]];
    }
    //每种组合的XY距离方差存储数组
    //vector<Vec2f> lines_xydistance(num_match);
    vector<float> lines_distance(num_match);
    //遍历所有可能的排列情况
    for (int nums = 0; nums < num_match; nums++)
    {
        int index_number = nums;
        //将组合序号与组合情况对应起来，计算每种组合的距离方差
        vector<int> corresepond_vec(vaild_index.size());
        for (int i = vaild_index.size() - 1; i > 0; i--)
        {
            corresepond_vec[i] = index_number % Mmatch1[i].als_index.size();
            index_number = int(index_number / Mmatch1[i].als_index.size());
        }
        corresepond_vec[0] = index_number;

        //根据导出的特征线组合计算距离方差
        float diff_distance = 0;
        float diff_xdistance = 0;
        float diff_ydistance = 0;
        //计算XY距离平均值
        float x_distance = 0;
        float y_distance = 0;
        for (int i = 0; i < corresepond_vec.size(); i++)
        {
            Vec2f tmep = Mmatch1[i].xy_drift[corresepond_vec[i]];
            x_distance += tmep[0];
            y_distance += tmep[1];
        }
        //计算偏移均值
        float x_mean = x_distance / corresepond_vec.size();
        float y_mean = y_distance / corresepond_vec.size();
        //计算XY方向的方差
        for (int i = 0; i < corresepond_vec.size(); i++)
        {
            Vec2f tmep = Mmatch1[i].xy_drift[corresepond_vec[i]];
            diff_xdistance += pow((x_mean - tmep[0]), 2);
            diff_ydistance += pow((y_mean - tmep[1]), 2);
        }
        diff_xdistance = diff_xdistance / corresepond_vec.size();
        diff_ydistance = diff_ydistance / corresepond_vec.size();
        diff_distance = sqrt(pow(diff_xdistance, 2) + pow(diff_ydistance, 2));
        lines_distance[nums] = diff_distance;
        //lines_xydistance[nums] = {diff_xdistance, diff_ydistance};
    }

    //从各种情况中找到距离方差最小的情况
    auto min_distance = min_element(lines_distance.begin(), lines_distance.end());
    int index_mindiff = distance(begin(lines_distance), min_distance);

    //对求得的最小距离方差进行偏移量优化
    int mindiffID = index_mindiff;
    //将组合序号与组合情况对应起来，计算每种组合的距离方差
    vector<int> mindiff_vec(vaild_index.size());
    for (int i = vaild_index.size() - 1; i > 0; i--)
    {
        mindiff_vec[i] = mindiffID % Mmatch1[i].als_index.size();
        mindiffID = int(mindiffID / Mmatch1[i].als_index.size());
    }
    mindiff_vec[0] = mindiffID;
    //利用带权平差法迭代精化
    float minx_distance = 0;
    float miny_distance = 0;
    for (int i = 0; i < mindiff_vec.size(); i++)
    {
        Vec2f tmep = Mmatch1[i].xy_drift[mindiff_vec[i]];
        minx_distance += tmep[0];
        miny_distance += tmep[1];
    }
    float x_mean = minx_distance / mindiff_vec.size();
    float y_mean = miny_distance / mindiff_vec.size();
    float xy_mean = sqrt(pow(x_mean, 2) + pow(y_mean, 2));
    //计算XY方向的方差
    //构建权矩阵
    vector<float> weight(mindiff_vec.size());
    vector<float> diff(mindiff_vec.size());
    float diff_minxydistance = 0;
    for (int i = 0; i < mindiff_vec.size(); i++)
    {
        Vec2f tmep = Mmatch1[i].xy_drift[mindiff_vec[i]];
        diff[i]= 1.0 / (sqrt(pow(x_mean - tmep[0], 2) + pow(y_mean - tmep[1], 2)) + eps);
        diff_minxydistance += diff[i];
    }
    for (int i = 0; i < mindiff_vec.size(); i++)
    {
        weight[i] = diff[i] / diff_minxydistance;
    }
    float x1_mean = 0;
    float y1_mean = 0;
    float xy1_mean = 0;
    for (int i = 0; i < mindiff_vec.size(); i++)
    {
        x1_mean += weight[i] * Mmatch1[i].xy_drift[mindiff_vec[i]][0];
        y1_mean += weight[i] * Mmatch1[i].xy_drift[mindiff_vec[i]][1];
    }
    x1_mean = x1_mean ;
    y1_mean = y1_mean ;
    xy1_mean = sqrt(pow(x1_mean, 2) + pow(y1_mean, 2));
    //构建权矩阵
    while (fabs(xy1_mean - xy_mean) > 0.1)
    {
        x_mean = x1_mean;
        y_mean = y1_mean;
        xy_mean = xy1_mean;
        //计算XY方向的方差
        float diff_minxydistance = 0;
        for (int i = 0; i < mindiff_vec.size(); i++)
        {
            Vec2f tmep = Mmatch1[i].xy_drift[mindiff_vec[i]];
            diff[i] = 1.0 / (sqrt(pow(x_mean - tmep[0], 2) + pow(y_mean - tmep[1], 2)) + eps);
            diff_minxydistance += diff[i];
        }
        for (int i = 0; i < mindiff_vec.size(); i++)
        {
            weight[i] = diff[i] / diff_minxydistance;
        }
        float x1_mean = 0;
        float y1_mean = 0;
        for (int i = 0; i < mindiff_vec.size(); i++)
        {
            x1_mean += weight[i] * Mmatch1[i].xy_drift[mindiff_vec[i]][0];
            y1_mean += weight[i] * Mmatch1[i].xy_drift[mindiff_vec[i]][1];
        }
        x1_mean = x1_mean ;
        y1_mean = y1_mean ;
        xy1_mean = sqrt(pow(x1_mean, 2) + pow(y1_mean, 2));
    }
    cout << "x方向上的偏移量：" << x1_mean << "Y方向上的偏移量：" << y1_mean << endl;
    //求Z坐标偏移量
    pcl::PointCloud<pcl::PointXYZI>::Ptr mls_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr als_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    float distance_threshold = 0.8;
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(200);
    //seg.setProbability(0.95);
    seg.setInputCloud(cloud_mls);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZI> extract_1;
    extract_1.setInputCloud(cloud_mls);
    extract_1.setIndices(inliers);
    extract_1.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract_1.filter(*mls_z);
    Eigen::Vector4f mls_centroid;
    pcl::compute3DCentroid(*mls_z, mls_centroid);

    pcl::ModelCoefficients::Ptr coefficients_1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_1(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg_1;
    // Optional
    seg_1.setOptimizeCoefficients(true);
    // Mandatory
    seg_1.setModelType(pcl::SACMODEL_PLANE);
    seg_1.setMethodType(pcl::SAC_RANSAC);
    seg_1.setDistanceThreshold(distance_threshold);
    seg_1.setMaxIterations(200);
    //seg.setProbability(0.95);
    seg_1.setInputCloud(Als_cloud);
    seg_1.segment(*inliers_1, *coefficients_1);
    pcl::ExtractIndices<pcl::PointXYZI> extract_2;
    extract_2.setInputCloud(Als_cloud);
    extract_2.setIndices(inliers_1);
    extract_2.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract_2.filter(*als_z);
    Eigen::Vector4f als_centroid;
    pcl::compute3DCentroid(*als_z, als_centroid);
    //点云校正
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rect(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_rect->resize(count_1);
    cloud_rect->width = count_1;
    cloud_rect->height = 1;
    cloud_rect->is_dense = false;
    for (int i = 0; i <  count_1; i++)
    {
        cloud_rect->points[i].x = Als_cloud->points[i].x + x1_mean;
        cloud_rect->points[i].y = Als_cloud->points[i].y + y1_mean;
        cloud_rect->points[i].z = Als_cloud->points[i].z + (mls_centroid[2] - als_centroid[2]);
        cloud_rect->points[i].intensity = Als_cloud->points[i].intensity;
    }
    pcl::visualization::PCLVisualizer viewer_2("视窗2");
    viewer_2.setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            target_color2(cloud_mls, 205, 92, 92);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            src_color2(cloud_rect, 144, 238, 238);

    viewer_2.addPointCloud<pcl::PointXYZI>(cloud_mls, target_color2, "sample2");//显示点云，其中fildColor为颜色显示
    viewer_2.addPointCloud<pcl::PointXYZI>(cloud_rect, src_color2, "sample_2");//显示点云，其中fildColor为颜色显示

    viewer_2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample2");//设置点云大小
    viewer_2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_2");//设置点云大小

    for (int i = 0; i < mindiff_vec.size(); i++)
    {
        char str[25];
        sprintf(str, "%d", i);
        float x_1 = mls_xyzline[vaild_index[i]].startpoint.x;
        float y_1 = mls_xyzline[vaild_index[i]].startpoint.y;
        float z_1 = mls_xyzline[vaild_index[i]].startpoint.z;
        float x_2 = als_xyzline[Mmatch1[i].als_index[mindiff_vec[i]]].startpoint.x + x1_mean;
        float y_2 = als_xyzline[Mmatch1[i].als_index[mindiff_vec[i]]].startpoint.y + y1_mean;
        float z_2 = als_xyzline[Mmatch1[i].als_index[mindiff_vec[i]]].startpoint.z +(mls_centroid[2] - als_centroid[2]);
        viewer_2.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(x_1, y_1, z_1), pcl::PointXYZ(x_2, y_2, z_1),0, 0, 255, str);
    }

    while (!viewer_2.wasStopped())
    {
        viewer_2.spinOnce();
    }
    waitKey(0);
    return (0);
}
