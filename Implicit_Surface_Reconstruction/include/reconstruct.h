//
// Created by Robotics_qi on 2020/7/29.
//

#ifndef MLS_STUDY_RECONSTRUCT_H
#define MLS_STUDY_RECONSTRUCT_H
#include <vector>

#define EPS 0.1

int mesh_size;
double xlo, xhi, ylo, yhi, zlo, zhi;
double xbinsize, ybinsize, zbinsize;

std::vector<std::vector<double> > pc_coords;
std::vector<std::vector<double> > pc_norm;

std::vector<double> rbf_weight;
std::vector<double> mesh_ls;

void readPointCloud(char *);
double distance(double, double, double, double, double, double);
double triharmonic_kernel(double);
void evaluateSDF();
void reconstructMesh();
void outputMesh();

#endif //MLS_STUDY_RECONSTRUCT_H
