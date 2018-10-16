#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <vector>


#include <opencv2/opencv.hpp>

#include "KinematicFeatures.h"

using namespace cv;
using namespace std;

#define PI  3.14159265358979323846;



float val_mean( float val_m, float val, float alpha);
float val_std( float std_m, float val_m, float val, float alpha);
float max_value( float  val_1 , float val_2 );
float min_value( float  val_1 , float val_2 );
float val_minimun(float val_min, float angle, float alpha);
float val_maximun(float val_max, float angle, float alpha);
vector<float> computeMeanFeat(vector<float> &vect_traj, float alpha);
vector<float> computeStdFeat(vector<float> &vect_traj, float alpha);
vector<float> computeMaxMinFeat(vector<float> &vect_traj, float alpha);



float computeRecursFeatByValue(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name, string recursive_name, float alpha)
{
    if(recursive_name.compare("mean_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeMeanFeat(vec_feature, alpha);
        return vec_recurs[vec_recurs.size()-1];
    }
    else if(recursive_name.compare("std_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeStdFeat(vec_feature, alpha);
        return vec_recurs[vec_recurs.size()-1];
    }
    else if(recursive_name.compare("maxmin_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeMaxMinFeat(vec_feature, alpha);
        return vec_recurs[vec_recurs.size()-1];
    }
}


vector<float> computeRecursFeatByVector(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name, string recursive_name, float alpha)
{
    if(recursive_name.compare("mean_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeMeanFeat(vec_feature, alpha);
        return vec_recurs;
    }
    else if(recursive_name.compare("std_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeStdFeat(vec_feature, alpha);
        return vec_recurs;
    }
    else if(recursive_name.compare("maxmin_f")  == 0)
    {
        vector<float> vec_feature, vec_recurs;
        vec_feature = computeKinFeatByVector(vect_traj_x, vect_traj_y, vect_traj_z, feature_name);
        vec_recurs  = computeMaxMinFeat(vec_feature, alpha);
        return vec_recurs;
    }
}



vector<float> computeMeanFeat(vector<float> &vect_feat, float alpha)
{
    vector<float> vectRet;
    vectRet.clear();
    float val_mean_temp = vect_feat[0];
    for(int t=1; t<vect_feat.size(); t++)
    {
        val_mean_temp = val_mean(val_mean_temp,vect_feat[t],alpha);
        vectRet.push_back(val_mean_temp);
    }
    return vectRet;
}

vector<float> computeStdFeat(vector<float> &vect_feat, float alpha)
{
    vector<float> vectRet;
    vectRet.clear();
    float val_mean_temp = vect_feat[0];
    float val_std_temp = 30; // according to literature is always a  "large value"
    for(int t=1; t<vect_feat.size(); t++)
    {
        val_mean_temp = val_mean(val_mean_temp,vect_feat[t],alpha);
        val_std_temp  = val_std(val_std_temp, val_mean_temp, vect_feat[t], alpha);
        vectRet.push_back(val_std_temp);
    }
    return vectRet;
}


vector<float>  computeMaxMinFeat(vector<float> &vect_feat, float alpha)
{
    vector<float> vectRet;
    vectRet.clear();
    float val_min = vect_feat[0], val_max = vect_feat[0];
    for(int t=1; t<vect_feat.size(); t++)
    {
        val_min =  val_minimun(val_min, vect_feat[t], alpha);
        val_max =  val_maximun(val_max, vect_feat[t], alpha);
        float val_grad =  val_max - val_min;
        vectRet.push_back(val_grad);
    }
    return vectRet;
}


float val_mean( float val_m, float val, float alpha)
{
    return alpha*val + (1-alpha)*(val_m);
}

float val_std( float std_m, float val_m, float val, float alpha)
{
    return (float)sqrt(pow(val_m - alpha*val, 2) + (1-alpha)*(std_m));
}

float max_value( float  val_1 , float val_2 )
{
    if(val_1> val_2)return val_1;
    else return val_2;
}
float min_value( float  val_1 , float val_2 )
{
    if(val_1< val_2)return val_1;
    else return val_2;
}
float val_minimun(float val_min, float angle, float alpha)
{
    return alpha*angle + (1-alpha)*min_value(angle, val_min);
}
float val_maximun(float val_max, float angle, float alpha)
{
    return alpha*angle + (1-alpha)*max_value(angle, val_max);
}

