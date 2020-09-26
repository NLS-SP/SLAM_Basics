//
// Created by Robotics_qi on 2020/9/23.
//

#include <iostream>

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <vector>
#include <random>
#include <iterator>

typedef struct{
    float x;
    float y;
}Point2D32f;

const int COUNT = 100;
const int inlierCnt = 40;
const int outlierCnt = COUNT - inlierCnt;

Point2D32f points[COUNT];

std::vector<double> data;

float lines[4] = {0.0};

void FitLine2D(Point2D32f *points, int count, float *line){

}

float Ransac(Point2D32f *points, size_t Cnt, float *lines){
    int numDataObjects = Cnt;
    int numForEstimate = Cnt * 0.1;
    int maxVoteCnt = 0;
    float tempLine[4];
    float inlinersPercentage = 0.0;

    // 随机抽取一定比例点
    int ransac_times = 5000;
    int *Chosen = new int[numDataObjects];

    Point2D32f *subPoints = new Point2D32f[numForEstimate];

    int pointCnt = 0, voteCnt = 0;
    for(int i = 0; i < ransac_times; i++){
        // 随机抽取数据用于模型拟合
        memset(Chosen, 0, numDataObjects * sizeof(int));
        int maxIndex = numDataObjects - 1;
        for(int j = 0; j < numForEstimate; j++){
            int selectedIndex = rand() % numDataObjects;
            Chosen[selectedIndex] = 1;
        }
        // 用于拟合
        pointCnt = 0;
        for(int k = 0; k < numDataObjects; k++){
            if(Chosen[k]){
                subPoints[pointCnt].x = points[k].x;
                subPoints[pointCnt].y = points[k].y;
                pointCnt++;
            }
        }
        FitLine2D(subPoints, pointCnt, tempLine);
        float a = tempLine[1] / tempLine[0];
        float b = tempLine[3] - a *tempLine[2];

        // 对拟合结果进行投票鉴定,选出最优结果
        voteCnt = 0;
        for(int k = 0; k < Cnt; k++){
            // 如果在直线上或附近
            if(abs(points[k].y - a * points[k].x - b) < 2)
                voteCnt++;
        }

        if(voteCnt > maxVoteCnt){
            maxVoteCnt = voteCnt;
            inlinersPercentage = (float)maxVoteCnt / Cnt;
            for(int m = 0; m < 4; m++)
                lines[m] = tempLine[m];
        }

        return inlinersPercentage;
    }
}

int main() {
    /**
     * !@brief The Gaussian Distribution.
     *  */
    for (double i = 0; i < 7; i++) data.push_back(i);

    // Define random generator with Gaussian distribution.
    const double mean = 0.0;
    const double stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    srand((unsigned int) time(0));

    float a = rand() % 100 / 5.0;
    float b = rand() % 100 / 5.0;

    std::cout << "The ground truth is: a: " << a << ", b: " << b << std::endl;

    // inliners.
    for (int i = 0; i < inlierCnt; i++) {
        points[i].x = i;
        points[i].y = i * a + b + dist(generator);
    }

    // outliers.
    for (int i = inlierCnt; i < COUNT; i++) {
        points[i].x = 20 + rand() % 100;
        points[i].y = 30 + rand() % 100;
    }


    // least square fit with only inliers.
    FitLine2D(points, inlierCnt, lines);
    a = lines[1] / lines[0];
    b = lines[3] - a * lines[2];
    std::cout << "Ransac fitting including outliers is: a: " << a << ", b: " << b << std::endl;
}