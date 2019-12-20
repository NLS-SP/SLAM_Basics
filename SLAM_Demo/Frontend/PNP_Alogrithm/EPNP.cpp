//
// Created by gatsby on 2019-02-26.
//

#include <iostream>
#include "EPNP.h"

EPNP::EPNP(void){
    maximum_number_of_correspondences = 0;
    number_of_correspondences = 0;

    pws = 0;
    us = 0;
    alphas = 0;
    pcs = 0;
}

EPNP::~EPNP(){
    delete []pws;
    delete []us;
    delete []alphas;
    delete []pcs;
}

void EPNP::set_internal_parameters(int n){
    if(maximum_number_of_correspondences < n){
        if(pws != 0) delete [] pws;
        if(us != 0) delete [] us;
        if(alphas != 0) delete [] alphas;
        if(pcs != 0) delete [] pcs;

        maximum_number_of_correspondences = n;
        pws = new double[3 * maximum_number_of_correspondences];
        us = new double[2 * maximum_number_of_correspondences];
        alphas = new double[4 * maximum_number_of_correspondences];
        pcs = new double[3 * maximum_number_of_correspondences];
    }
}

void EPNP::reset_correspondences(void){
    number_of_correspondences = 0;
}

void EPNP::add_correspondence(double X, double Y, double Z, double u, double v){
    pws[3 * number_of_correspondences    ] = X;
    pws[3 * number_of_correspondences + 1] = Y;
    pws[3 * number_of_correspondences + 2] = Z;

    us[2 * number_of_correspondences     ] = u;
    us[2 * number_of_correspondences  + 1] = v;
    number_of_correspondences++;
}

void EPNP::choose_control_points(void){
    // Take C0 as the reference points centroid;
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            cws[0][j] += pws[3 * i + j];

    for(int j = 0; j < 3; j++)
        cws[0][j] /= number_of_correspondences;

    // Take C1, C2 and C3 from PCA on the reference points;
    CvMat *PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

    double pw0tpw0[3*3], dc[3], uct[3*3];
    CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
    CvMat DC = cvMat(3, 1, CV_64F, dc);
    CvMat UCt = cvMat(3, 3, CV_64F, uct);

    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j];

    cv::cvMulTransposed(PW0, &PW0tPW0, 1);
    cvSVD(&PW0tPW0, &DC, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

    cvReleaseMat(&PW0);

    for(int i = 1; i < 4; i++){
        double k = sqrt(dc[i-1]/number_of_correspondences);
        for(int j = 0; j < 3; j++)
            cws[i][j] = cws[0][j] + k * uct[3 * (i-1) +j];
    }
}