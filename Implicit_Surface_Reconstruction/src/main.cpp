//
// Created by Robotics_qi on 2020/7/29.
//


#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cstring>
#include <Eigen/Dense>
#include "reconstruct.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    // Input: point cloud filename
    // Input: mesh size, box boundaries

    // Output: the implicit surface representation in a VTP file. (mesh points with SDFs)

    if(argc < 9)
    {
        cout<<"Usage: <pointcloud_file> <mesh_size> <low_x> <high_x> <low_y> <high_y> <low_z> <high_z>";
        return 0;
    }

    cout<<endl;
    cout<<"Reading the input file...";
    // Read input file
    readPointCloud(argv[1]);

    cout<<endl<<"Reading parameters...";
    // Set parameters for the output mesh
    mesh_size = atoi(argv[2]);
    xlo = atof(argv[3]);
    xhi = atof(argv[4]);
    ylo = atof(argv[5]);
    yhi = atof(argv[6]);
    zlo = atof(argv[7]);
    zhi = atof(argv[8]);

    xbinsize = (xhi - xlo) / mesh_size;
    ybinsize = (yhi - ylo) / mesh_size;
    zbinsize = (zhi - zlo) / mesh_size;

    cout<<endl<<"Evaluating the SDF...";
    // Evaluate the signed distance functions
    evaluateSDF();

    cout<<endl<<"Reconstructing mesh...";
    reconstructMesh();
    cout<<endl<<"Writing mesh structure to file...";
    outputMesh();
    cout<<endl<<"Done."<<endl;

    return 0;
}

void readPointCloud(char* filename){
    int i;
    int nump = 0;
    bool done_flag = false;
    std::string line, temp;
    std::ifstream infile;
    std::stringstream iss;

    infile.open(filename, std::ios::in);
    while(!infile.eof() && !done_flag){
        infile >> line;
        if(strcmp(line.c_str(), "<piece") == 0){
            infile >> line;
            iss.str(line);
            getline(iss, temp, '\"');
            getline(iss, temp, '\"');
            nump = atoi(temp.c_str());
            pc_coords.resize(nump, std::vector<double>(3, 0.0));
            pc_norm.resize(nump, std::vector<double>(3, 0.0));
        }else if(strcmp(line.c_str(), "<Points>")){
            double rx, ry, rz;
            for(i = 0; i < 5; i++) infile >> line;
            for(i = 0; i < nump; i++){
                infile >> rx >> ry >> rz;
                pc_coords[i][0] = rx;
                pc_coords[i][1] = ry;
                pc_coords[i][2] = rz;
            }
        }else if(strcmp(line.c_str(), "Name=\"Normals\"") == 0){
            double nx, ny, nz;
            infile >> line >> line;
            for(i = 0; i < nump; i++){
                infile >> nx >> ny >> nz;
                pc_norm[i][0] = nx;
                pc_norm[i][1] = ny;
                pc_norm[i][2] = nz;
            }
            done_flag = true;
        }
    }
    infile.close();
    return;
}

double distance(double x1, double y1, double z1, double x2, double y2, double z2){
    double d;
    d = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
    return d;
}

double triharmonic_kernel(double x){
    return x * x * x;
}

// Calculate the weights;
void evaluateSDF(){
    int i, j;
    int nump = pc_coords.size();
    double x1, y1, z1, x2, y2, z2;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
    Eigen::Matrix<double, Eigen::Dynamic, 1> d;

    K.resize(2 * nump, 2 * nump);
    d.resize(2 * nump, Eigen::NoChange);

    for(i = 0; i < 2 * nump; i++){
        if(i < nump){
            x1 = pc_coords[i][0];
            y1 = pc_coords[i][1];
            z1 = pc_coords[i][2];
            d(i) = 0.0;
        }else{
            x1 = pc_coords[i - nump][0] + EPS * pc_norm[i - nump][0];
            y1 = pc_coords[i - nump][1] + EPS * pc_norm[i - nump][1];
            z1 = pc_coords[i - nump][2] + EPS * pc_norm[i - nump][2];
            d(i) = EPS;
        }
        for(j = 0; j < 2 * nump; j++){
            if(j < nump){
                x2 = pc_coords[j][0];
                y2 = pc_coords[j][1];
                z2 = pc_coords[j][2];
            }else{
                x2 = pc_coords[j - nump][0] + EPS * pc_norm[j - nump][0];
                y2 = pc_coords[j - nump][1] + EPS * pc_norm[j - nump][1];
                z2 = pc_coords[j - nump][2] + EPS * pc_norm[j - nump][2];
            }
            K(i, j) = triharmonic_kernel(distance(x1, y1, z1, x2, y2, z2));
        }
    }
    Eigen::Matrix<double, Eigen::Dynamic, 1> w = K.inverse() * d;
    w.resize(2 * nump, Eigen::NoChange);

    rbf_weight.resize(2 * nump);

    for(i = 0; i < 2 * nump; i++)
        rbf_weight[i] = w[i];

    return;
}

// Reconstruct Mesh based on the calculated weights.
void reconstructMesh(){
    int i, j, k, l;
    int nump = pc_coords.size();
    long index;
    double mx, my, mz, mphi;

    mesh_ls.resize(mesh_size * mesh_size * mesh_size, 0);

    for(i = 0; i < mesh_size; i++){
        mx = xlo + xbinsize * i;
        for(j = 0; j < mesh_size; j++){
            my = ylo + ybinsize * j;
            for(k = 0; k < mesh_size; k++){
                mz = zlo + zbinsize * k;
                mphi = 0.0;
                for(l = 0; l < nump; l++)
                    mphi += rbf_weight[l] * triharmonic_kernel(distance(mx, my, mz, pc_coords[l][0], pc_coords[l][1], pc_coords[l][2]));
                for(l = 0; l < nump; l++)
                    mphi += rbf_weight[l+nump] * triharmonic_kernel(distance(mx, my, mz, pc_coords[l][0] + EPS * pc_norm[l][0], pc_coords[l][1] + EPS * pc_norm[l][1], pc_coords[l][2] + EPS * pc_norm[l][2]));
                index = k * mesh_size * mesh_size + j * mesh_size +i;
                mesh_ls[index] = mphi;
            }
        }
    }
    return;
}

// Write output file in VTK format
void outputMesh()
{
    int i;
    ofstream outfile;

    outfile.open("output.vtk", ios::out);
    outfile<<"# vtk DataFile Version 2.0"<<endl;
    outfile<<"Level set data"<<endl;
    outfile<<"ASCII"<<endl;
    outfile<<"DATASET RECTILINEAR_GRID"<<endl;
    outfile<<"DIMENSIONS "<<mesh_size<<" "<<mesh_size<<" "<<mesh_size<<endl;
    outfile<<"X_COORDINATES "<<mesh_size<<" float"<<endl;
    for(i = 0;i < mesh_size;i++)
    {
        outfile<<i*xbinsize + xlo<<endl;
    }
    outfile<<"Y_COORDINATES "<<mesh_size<<" float"<<endl;
    for(i = 0;i < mesh_size;i++)
    {
        outfile<<i*ybinsize + ylo<<endl;
    }
    outfile<<"Z_COORDINATES "<<mesh_size<<" float"<<endl;
    for(i = 0;i < mesh_size;i++)
    {
        outfile<<i*zbinsize + zlo<<endl;
    }
    outfile<<"POINT_DATA "<<mesh_ls.size()<<endl;
    outfile<<"SCALARS ls_phi float 1"<<endl;
    outfile<<"LOOKUP_TABLE default"<<endl;
    for(i = 0;i < mesh_ls.size();i++)
    {
        outfile<<mesh_ls[i]<<endl;
    }
    outfile.close();
}