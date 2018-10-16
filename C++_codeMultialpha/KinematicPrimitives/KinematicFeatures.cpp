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
//modified
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define PI  3.14159265358979323846;

void compute_CurvTorsDevsSigned(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z,
                                vector<float> &vect_curv,
                                vector<float> &vect_tor,
                                vector<float> &vect_devCurv,
                                vector<float> &vect_devTors);

void compute_CurvTorsDevs(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z,
                          vector<float> &vect_curv,  vector<float> &vect_tor,
                          vector<float> &vect_devCurv, vector<float> &vect_devTors);

void compute_onlyCurv(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z, vector<float> &vect_curv);
void compute_onlyCurvSigned(vector<float> &vect_x, vector<float> &vect_y,  vector<float> &vect_z, vector<float> &vect_curv);

void compute_onlyDevCurvSigned(vector<float> &vect_x, vector<float> &vect_y,
                               vector<float> &vect_z, vector<float> &vect_devcurv);
void compute_Speed(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> &feat_normVel);
void compute_Velxy(vector<float> &v_x, vector<float> &v_y, vector<float> &vec_vel_x, vector<float> &vec_vel_y, vector<float> &vec_vel_xS, vector<float> &vec_vel_yS);
void compute_Angle(vector<float> &vect_x, vector<float> &vect_y,  vector<float> &vectRet);
void compute_AnglebyBeans(vector<float> &vect_angle,  vector<float> &vectRet, int bines);

float val_euclDistance3D(float val_ax, float val_ay, float val_az,  float val_bx, float val_by,  float val_bz);
float val_euclDistance1D(float val_ax, float val_bx);
float val_curv(float a, float b, float c, float s, float curv_temp);
float val_tors(float d, float e, float f, float curv, float h_max, float tors_temp);
float scale_( float y_min, float y_max, float value_in );

float val_degree(float val_x, float val_y);
float val_Distance1D(float val_a , float val_aprev);

vector<float> vec_euclnom(vector<float> vecx, vector<float> vecy);
vector<float> vec_der1D(vector<float> vec_);
vector<float> multiTwovect(vector<float> vec_v_norm, vector<float> vec_a_norm);
void computeAcelGen(vector<float> &acelT, vector<float> &acelN,  vector<float> &tan_x, vector<float> &tan_y, vector<float> &norm_x,
                    vector<float> &norm_y, vector<float> &acelgenX, vector<float> &acelgenY);




vector<float> computeKinFeatByVector(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name)
{


    ///---------------------------------------------------------------------------------
    ///----------------------  curv tos and devs Classical  ----------------------------
    ///---------------------------------------------------------------------------------
    if(feature_name.compare("feat_curv")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
//        compute_CurvTorsDevs(vect_traj_x, vect_traj_y, vect_traj_z,
//                                  feat_vect,  vect_temp,
//                                  vect_temp, vect_temp);
        compute_onlyCurv(vect_traj_x, vect_traj_y, vect_traj_z, feat_vect);
        //feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_devcurv")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
//        compute_CurvTorsDevs(vect_traj_x, vect_traj_y, vect_traj_z,
//                                  vect_temp, vect_temp,
//                                  feat_vect, vect_temp);
        compute_onlyDevCurvSigned(vect_traj_x, vect_traj_y, vect_traj_z, feat_vect);
//        cout<< "size traj "<< vect_traj_x.size() << " size feat "<< feat_vect.size() << endl;

//        feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        return feat_vect;

    }

    ///---------------------------------------------------------------------------------
    ///----------------------  curv tos and devs Signed --------------------------------
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_curvSigned")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
//        compute_CurvTorsDevsSigned(vect_traj_x, vect_traj_y, vect_traj_z,
//                                  feat_vect,  vect_temp,
//                                  vect_temp, vect_temp);
        compute_onlyCurvSigned(vect_traj_x, vect_traj_y, vect_traj_z, feat_vect);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_torsSigned")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
        compute_CurvTorsDevsSigned(vect_traj_x, vect_traj_y, vect_traj_z,
                                   vect_temp, feat_vect,
                                   vect_temp, vect_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_devcurvSigned")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
        compute_CurvTorsDevsSigned(vect_traj_x, vect_traj_y, vect_traj_z,
                                   vect_temp, vect_temp,
                                   feat_vect, vect_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_devtorsSigned")  == 0)
    {
        vector<float> feat_vect;
        vector<float> vect_temp;
        compute_CurvTorsDevsSigned(vect_traj_x, vect_traj_y, vect_traj_z,
                                   vect_temp, vect_temp,
                                   vect_temp, feat_vect);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------
    ///----------------------  vel and acel normalized  --------------------------------
    ///---------------------------------------------------------------------------------
    else if(feature_name.compare("feat_speed")  == 0)
    {
        ///||v(t)||
        vector<float> feat_vect, vec_temp;
        compute_Speed(vect_traj_x, vect_traj_y, feat_vect);
        return feat_vect;
    }

    else if(feature_name.compare("feat_Tangx")  == 0)
    {
        vector<float> feat_vect, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y,  feat_vect, vec_temp, vec_temp, vec_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_Tangy")  == 0)
    {

        vector<float> feat_vect, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y,  vec_temp, feat_vect, vec_temp, vec_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_acelx")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp,  vec_velx, vec_vely);
        compute_Velxy(vec_velx, vec_vely,  feat_vect, vec_temp, vec_temp, vec_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_acely")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp, vec_velx, vec_vely);
        compute_Velxy(vec_velx, vec_vely, vec_temp, feat_vect, vec_temp, vec_temp);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    ///----------------------  Theta patterns variations  ------------------------------------
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_theta")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp, vec_velx, vec_vely);
        compute_Angle(vec_velx, vec_vely, feat_vect);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_thetaAcel")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_acelx, vec_acely, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp, vec_velx, vec_vely);
        compute_Velxy(vec_velx, vec_vely, vec_temp, vec_temp, vec_acelx, vec_acely);
        compute_Angle(vec_acelx, vec_acely, feat_vect);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_ThetaBin")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_temp, vect_angle;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp, vec_velx, vec_vely);
        compute_Angle(vec_velx, vec_vely, vect_angle);
        compute_AnglebyBeans(vect_angle,  feat_vect, 31);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_ThetaBinTang")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_temp, vect_angle;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_velx, vec_vely, vec_temp, vec_temp);
        compute_Angle(vec_velx, vec_vely, vect_angle);
        compute_AnglebyBeans(vect_angle,  feat_vect, 31);
        return feat_vect;

    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_ThetaBinNorm")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_acelx, vec_acely, vec_temp, vect_angle;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_velx, vec_vely, vec_temp, vec_temp);
        compute_Velxy(vec_velx, vec_vely, vec_acelx, vec_acely, vec_temp, vec_temp);
        compute_Angle(vec_acelx, vec_acely, vect_angle);
        compute_AnglebyBeans(vect_angle,  feat_vect, 31);
        return feat_vect;
    }
    ///---------------------------------------------------------------------------------------
    else if(feature_name.compare("feat_thetaAcelNorm")  == 0)
    {
        vector<float> feat_vect, vec_velx, vec_vely, vec_acelx, vec_acely, vec_temp;
        compute_Velxy(vect_traj_x, vect_traj_y, vec_temp, vec_temp, vec_velx, vec_vely);
        compute_Velxy(vec_velx, vec_vely, vec_acelx, vec_acely, vec_temp, vec_temp);
        compute_Angle(vec_acelx, vec_acely, feat_vect);
        return feat_vect;
    }
    else if(feature_name.compare("feat_ThetaBinAcelGeneral")  == 0)
    {

        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, tan_x, tan_y, norm_x, norm_y, vec_v_norm, acelT,
               acelN, acel_xs, acel_ys, vec_a_norm, vec_accelGenX,vec_accelGenY, vect_angle;
        compute_Velxy(vect_traj_x, vect_traj_y,  tan_x, tan_y, vec_xs, vec_ys);
        compute_Velxy(tan_x, tan_y, norm_x, norm_y, acel_xs, acel_ys);
        vec_v_norm = vec_euclnom(vec_xs, vec_ys);
        vec_a_norm = vec_euclnom(acel_xs, acel_ys);
        vec_v_norm.erase(vec_v_norm.begin());
        acelT = vec_der1D(vec_v_norm);
        acelN =  multiTwovect(vec_v_norm, vec_a_norm);
        acelN.erase(acelN.begin());
        norm_x.erase(norm_x.begin());
        norm_y.erase(norm_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        computeAcelGen(acelT, acelN,  tan_x, tan_y, norm_x, norm_y, vec_accelGenX,vec_accelGenY);

        compute_Angle(vec_accelGenX,vec_accelGenY, vect_angle);
        compute_AnglebyBeans(vect_angle,  feat_vect, 31);
        return feat_vect;
    }

    ///---------------------------------------------------------------------------------------
    ///----------------------  Acceleration and normal components patterns variations  -------
    ///---------------------------------------------------------------------------------------

    else if(feature_name.compare("feat_acelT")  == 0)
    {
        /// at = d(||v(t)||)/dt
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, vec_v_norm;
        compute_Velxy(vect_traj_x, vect_traj_y,  vec_temp, vec_temp, vec_xs, vec_ys);
        vec_v_norm = vec_euclnom(vec_xs, vec_ys);
        feat_vect = vec_der1D(vec_v_norm);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        //feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        return feat_vect;
    }


    else if(feature_name.compare("feat_acelN")  == 0)
    {
        /// an = ||v(t)||.||T'(t)||
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, vec_x, vec_y, vec_v_norm, acel_xs,acel_ys, vec_a_norm;
        compute_Velxy(vect_traj_x, vect_traj_y,  vec_x, vec_y, vec_xs, vec_ys);
        compute_Velxy(vec_x, vec_y,  vec_temp, vec_temp, acel_xs, acel_ys);
        vec_v_norm = vec_euclnom(vec_xs, vec_ys);
        vec_a_norm = vec_euclnom(acel_xs, acel_ys);
        /// vec_v_norm[i+1] = porque acel es un tiempo mas
        vec_v_norm.erase(vec_v_norm.begin());
        feat_vect = multiTwovect(vec_v_norm, vec_a_norm);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        //feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        return feat_vect;

    }
    else if(feature_name.compare("feat_acelGeneralX")  == 0)
    {
        /// ag= at(t)T(t) + an(t)N(t)
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, tan_x, tan_y, norm_x, norm_y, vec_v_norm, acelT, acelN, acel_xs, acel_ys, vec_a_norm;
        compute_Velxy(vect_traj_x, vect_traj_y,  tan_x, tan_y, vec_xs, vec_ys);
        compute_Velxy(tan_x, tan_y, norm_x, norm_y, acel_xs, acel_ys);
        vec_v_norm = vec_euclnom(vec_xs, vec_ys);
        vec_a_norm = vec_euclnom(acel_xs, acel_ys);
        vec_v_norm.erase(vec_v_norm.begin());
        acelT = vec_der1D(vec_v_norm);
        acelN =  multiTwovect(vec_v_norm, vec_a_norm);
        acelN.erase(acelN.begin());
        norm_x.erase(norm_x.begin());
        norm_y.erase(norm_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        computeAcelGen(acelT, acelN,  tan_x, tan_y, norm_x, norm_y, feat_vect, vec_temp);
        return feat_vect;

    }
    else if(feature_name.compare("feat_acelGeneralY")  == 0)
    {
        /// ag= at(t)T(t) + an(t)N(t)
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, tan_x, tan_y, norm_x, norm_y, vec_v_norm, acelT, acelN, acel_xs, acel_ys, vec_a_norm;
        compute_Velxy(vect_traj_x, vect_traj_y,  tan_x, tan_y, vec_xs, vec_ys);
        compute_Velxy(tan_x, tan_y, norm_x, norm_y, acel_xs, acel_ys);
        vec_v_norm = vec_euclnom(vec_xs, vec_ys);
        vec_a_norm = vec_euclnom(acel_xs, acel_ys);
        vec_v_norm.erase(vec_v_norm.begin());
        acelT = vec_der1D(vec_v_norm);
        acelN =  multiTwovect(vec_v_norm, vec_a_norm);
        acelN.erase(acelN.begin());
        norm_x.erase(norm_x.begin());
        norm_y.erase(norm_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        tan_x.erase(tan_x.begin());
        tan_y.erase(tan_y.begin());
        computeAcelGen(acelT, acelN,  tan_x, tan_y, norm_x, norm_y, vec_temp, feat_vect);
        return feat_vect;
    }
    else if(feature_name.compare("feat_Normalx")  == 0)
    {
        /// componentes Normales de N(t) = T'(t)/||T'(t)||
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, vec_x, vec_y;
        compute_Velxy(vect_traj_x, vect_traj_y,  vec_x, vec_y, vec_xs, vec_ys);
        compute_Velxy(vec_x, vec_y,  feat_vect, vec_temp, vec_temp, vec_temp);
        // feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        return feat_vect;

    }
    else if(feature_name.compare("feat_Normaly")  == 0)
    {
        /// componentes Normales de N(t) = T'(t)/||T'(t)||
        vector<float> feat_vect, vec_temp, vec_xs, vec_ys, vec_x, vec_y;
        compute_Velxy(vect_traj_x, vect_traj_y,  vec_x, vec_y, vec_xs, vec_ys);
        compute_Velxy(vec_x, vec_y,vec_temp, feat_vect, vec_temp, vec_temp);
        //feat_vect.push_back(feat_vect[feat_vect.size()-1]);
        feat_vect.insert(feat_vect.begin(), feat_vect[0]);
        return feat_vect;
    }

}




float val_euclDistance3D(float val_ax, float val_ay, float val_az,  float val_bx, float val_by,  float val_bz)
{
    return sqrt(pow((val_ax-val_bx),2) + pow((val_ay-val_by),2)+ pow((val_az-val_bz),2));
}

float val_euclDistance2D(float val_ax, float val_ay,  float val_bx, float val_by)
{
    return sqrt(pow((val_ax-val_bx),2) + pow((val_ay-val_by),2));
}

float val_euclDistance1D(float val_ax, float val_bx)
{
    return sqrt(pow((val_ax-val_bx),2));
}

float val_euclnom(float val_ax, float val_ay)
{
    return sqrt(pow((val_ax),2) + pow((val_ay),2) );
}

float val_Distance1D(float val_a , float val_aprev)
{
    return val_a - val_aprev;

}

vector<float> multiTwovect(vector<float> vec_v_norm, vector<float> vec_a_norm)
{
    vector<float> vec_multi;

    for(int i=0; i<vec_a_norm.size(); i++)
    {
        vec_multi.push_back(vec_v_norm[i]*vec_a_norm[i]);
    }
    return vec_multi;

}
vector<float> vec_der1D(vector<float> vec_)
{
    vector<float> vec_d1D;
    for(int i=1; i< vec_.size(); i++)
    {
        vec_d1D.push_back(val_Distance1D(vec_[i],vec_[i-1]));
    }
    return vec_d1D;
}

vector<float> vec_euclnom(vector<float> vecx, vector<float> vecy)
{
    vector<float> vec_en;
    for(int i=0; i< vecx.size(); i++)
    {
        vec_en.push_back(val_euclnom(vecx[i],vecy[i]));
    }
    return vec_en;
}


float val_curv(float a, float b, float c, float s, float curv_temp)
{
    if(a*b*c!=0 && (s*(s-a)*(s-b)*(s-c)) > 0)
    {
        return 4*((sqrt(s*(s-a)*(s-b)*(s-c)))/(a*b*c));
    }
    else
    {
        return 1;
    }
}

float val_tors(float d, float e, float f, float curv, float h_max, float tors_temp)
{
    if(d*e*f*curv!=0)
    {
        return 6*(h_max)/(d*e*f*curv);
    }
    else
    {
        return 0;
    }
}


float val_degree(float val_x, float val_y)
{

    float angle= fastAtan2(val_y, val_x);

    //if(angle<0){ angle = 360 + (angle * 180) / PI;}else{ angle = (angle * 180) / PI;}
    return angle;
}


void compute_onlyCurv(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z, vector<float> &vect_curv)
{
    float curv_temp=0;
    for(int t=2; t<vect_y.size(); t++)
    {
        float a = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float b = val_euclDistance3D(vect_x[t-0], vect_y[t-0], vect_z[t-0], vect_x[t-1], vect_y[t-1], vect_z[t-1]);
        float c = val_euclDistance3D(vect_x[t-0], vect_y[t-0], vect_z[t-0], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float s = (a+b+c)/2;
        float curv = val_curv(a, b, c, s, curv_temp);
        vect_curv.push_back(curv);
        curv_temp = curv;
    }

}

void compute_onlyCurvSigned(vector<float> &vect_x, vector<float> &vect_y,  vector<float> &vect_z, vector<float> &vect_curv)
{
    float curv_temp=0;
    for(int t=2; t<vect_y.size(); t++)
    {
        float a = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float b = val_euclDistance3D(vect_x[t-0], vect_y[t-0], vect_z[t-0], vect_x[t-1], vect_y[t-1], vect_z[t-1]);
        float c = val_euclDistance3D(vect_x[t-0], vect_y[t-0], vect_z[t-0], vect_x[t-2], vect_y[t-2], vect_z[t-2]);

        float curv =   (val_Distance1D(vect_x[t-1], vect_x[t-2])*val_Distance1D(vect_y[t-1], vect_y[t-0]))-
                       (val_Distance1D(vect_y[t-1], vect_y[t-2])*val_Distance1D(vect_x[t-1], vect_x[t-0]));
        //float curv = 0;
        //if(a*b*c != 0){curv =(float)(((float)2.0)*areaTriang2)/(a*b*c);}
        //else{curv = curv_temp;}
        vect_curv.push_back(curv);
        curv_temp = curv;
    }

}


void compute_onlyDevCurvSigned(vector<float> &vect_x, vector<float> &vect_y,
                               vector<float> &vect_z, vector<float> &vect_devcurv)
{
    float curv_temp=0;
    float dev_curv_temp=0;

    if(vect_x.size()>= 4)
    {
        for(int t=4; t< vect_y.size(); t++ )
        {

            float a = val_euclDistance3D(vect_x[t-2], vect_y[t-2], vect_z[t-2], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
            float b = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
            float c = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
            float d = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-1], vect_y[t-1], vect_z[t-1]);
            float e = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-2], vect_y[t-2], vect_z[t-2]);
            float f = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-3], vect_y[t-3], vect_z[t-3]);
            float g = val_euclDistance3D(vect_x[t-3], vect_y[t-3], vect_z[t-3], vect_x[t-4], vect_y[t-4], vect_z[t-4]);

            float s = (a+b+c)/2;
            float curv = val_curv(a, b, c, s, curv_temp);

            float dev_curv =0;
            if((2*a+ 2*b + d + g)!=0)
            {
                dev_curv = 3*((curv - curv_temp)/(2*a+ 2*b + d + g));
            }
            else
            {
                dev_curv=dev_curv_temp;
            }
            vect_devcurv.push_back(dev_curv);
            curv_temp = curv;
            dev_curv_temp =dev_curv;
        }
    }
    else
    {

        for(int i=0; i<(vect_x.size()-4); i++)
            vect_devcurv.push_back(0);
    }

}


void compute_CurvTorsDevsSigned(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z,
                                vector<float> &vect_curv,
                                vector<float> &vect_tor,
                                vector<float> &vect_devCurv,
                                vector<float> &vect_devTors)
{

    float curv_temp=0;
    float tors_temp=0;
    float dev_curv_temp=0;
    float dev_tors_temp=0;


    for(int t=4; t< vect_y.size(); t++ )  //curvature p(t-2)
    {
        float a = val_euclDistance3D(vect_x[t-2], vect_y[t-2], vect_z[t-2], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float b = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float c = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float d = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-1], vect_y[t-1], vect_z[t-1]);
        float e = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float f = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float g = val_euclDistance3D(vect_x[t-3], vect_y[t-3], vect_z[t-3], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float n = val_euclDistance3D(vect_x[t-2], vect_y[t-2], vect_z[t-2], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float m = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float s = (a+b+c)/2;
        float sb = (e+b+d)/2;


        float areaTriang2 =   (val_Distance1D(vect_x[t-2], vect_x[t-3])*val_Distance1D(vect_y[t-2], vect_y[t-1]))-
                              (val_Distance1D(vect_y[t-2], vect_y[t-3])*val_Distance1D(vect_x[t-2], vect_x[t-1]));

        areaTriang2 = areaTriang2/(float)2;

        float curv = 0;
        if(a*b*c != 0)
        {
            curv =(float)(((float)4.0)*areaTriang2)/(a*b*c);
        }



        Mat mat_detHmax = (Mat_<float>(4,4) <<   vect_x[t-2],  vect_y[t-2], vect_z[t-2], 1,
                           vect_x[t-3], vect_y[t-3], vect_z[t-3], 1,
                           vect_x[t-1], vect_y[t-1], vect_z[t-1], 1,
                           vect_x[t-0], vect_y[t-0], vect_z[t-0], 1
                          );
        Mat mat_detHmin= (Mat_<float>(4,4) <<  vect_x[t-2], vect_y[t-2], vect_z[t-2], 1,
                          vect_x[t-1], vect_y[t-1], vect_z[t-1], 1,
                          vect_x[t-3], vect_y[t-3], vect_z[t-3], 1,
                          vect_x[t-4], vect_y[t-4], vect_z[t-4], 1
                         );

        float h_max = determinant(mat_detHmax)/(2*areaTriang2);
        float h_min = determinant(mat_detHmin)/(2*areaTriang2);
        float tors1=  val_tors( d,  e,  f,  curv,  h_max, tors_temp);
        float tors2 =  val_tors( g,  n,  m,  curv,  h_min, tors_temp);
        float tors = (tors1+ tors2)/2;


        if(t>4)
        {
            float dev_curv =0;
            if((2*a+ 2*b + d + g)!=0)
            {
                dev_curv = 3*((curv - curv_temp)/(2*a+ 2*b + d + g));
            }


            float h= 2*s/c;
            float r= 2*a +2*b - 2*d -3*h +g;
            float dev_tors =0;


            if((2*a +2*b+2*d+h+g)!=0 && curv != 0)
            {
                float numerador   = tors - tors_temp + r*((tors*dev_curv)/(6*curv));
                float denominador = 2*a +2*b+2*d+h+g;
                dev_tors = 4*(numerador/denominador);
            }


            vect_devCurv.push_back(dev_curv);
            vect_devTors.push_back(dev_tors);
            dev_curv_temp=dev_curv;
            dev_tors_temp=dev_tors;
        }

        vect_curv.push_back(curv);
        vect_tor.push_back(tors);
        curv_temp = curv;
        tors_temp = tors;
    }
}




void compute_CurvTorsDevs(vector<float> &vect_x, vector<float> &vect_y, vector<float> &vect_z,
                          vector<float> &vect_curv,
                          vector<float> &vect_tor,
                          vector<float> &vect_devCurv,
                          vector<float> &vect_devTors)
{

    float curv_temp=0;
    float tors_temp=0;
    float dev_curv_temp=0;
    float dev_tors_temp=0;


    for(int t=4; t< vect_y.size(); t++ )  //curvature p(t-2)
    {
        float a = val_euclDistance3D(vect_x[t-2], vect_y[t-2], vect_z[t-2], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float b = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float c = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float d = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-1], vect_y[t-1], vect_z[t-1]);
        float e = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-2], vect_y[t-2], vect_z[t-2]);
        float f = val_euclDistance3D(vect_x[t],   vect_y[t],   vect_z[t],   vect_x[t-3], vect_y[t-3], vect_z[t-3]);
        float g = val_euclDistance3D(vect_x[t-3], vect_y[t-3], vect_z[t-3], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float n = val_euclDistance3D(vect_x[t-2], vect_y[t-2], vect_z[t-2], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float m = val_euclDistance3D(vect_x[t-1], vect_y[t-1], vect_z[t-1], vect_x[t-4], vect_y[t-4], vect_z[t-4]);
        float s = (a+b+c)/2;
        float sb = (e+b+d)/2;
        float curv = val_curv(a, b, c, s, curv_temp );
        float areaABC = sqrt(s*(s-a)*(s-b)*(s-c));

        Mat mat_detHmax = (Mat_<float>(4,4) <<   vect_x[t-2],  vect_y[t-2], vect_z[t-2], 1,
                           vect_x[t-3], vect_y[t-3], vect_z[t-3], 1,
                           vect_x[t-1], vect_y[t-1], vect_z[t-1], 1,
                           vect_x[t-0], vect_y[t-0], vect_z[t-0], 1
                          );
        Mat mat_detHmin= (Mat_<float>(4,4) <<  vect_x[t-2], vect_y[t-2], vect_z[t-2], 1,
                          vect_x[t-1], vect_y[t-1], vect_z[t-1], 1,
                          vect_x[t-3], vect_y[t-3], vect_z[t-3], 1,
                          vect_x[t-4], vect_y[t-4], vect_z[t-4], 1
                         );

        float h_max = determinant(mat_detHmax)/(2*areaABC);
        float h_min = determinant(mat_detHmin)/(2*areaABC);
        float tors1=  val_tors( d,  e,  f,  curv,  h_max, tors_temp);
        float tors2 =  val_tors( g,  n,  m,  curv,  h_min, tors_temp);
        float tors = (tors1+ tors2)/2;




        if(t>4)
        {
            float dev_curv =0;
            if((2*a+ 2*b + d + g)!=0)
            {
                dev_curv = 3*((curv - curv_temp)/(2*a+ 2*b + d + g));
            }
            float h= 2*s/c;
            float r= 2*a +2*b - 2*d -3*h +g;
            float dev_tors =0;

            if((2*a +2*b+2*d+h+g)!=0 && curv != 0)
            {
                float numerador   = tors - tors_temp + r*((tors*dev_curv)/(6*curv));
                float denominador = 2*a +2*b+2*d+h+g;
                dev_tors = 4*(numerador/denominador);
            }

            vect_devCurv.push_back(dev_curv);
            vect_devTors.push_back(dev_tors);

            dev_curv_temp=dev_curv;
            dev_tors_temp=dev_tors;
        }
        vect_curv.push_back(curv);
        vect_tor.push_back(tors);
        tors_temp = tors;
        curv_temp = curv;
    }
}


void computeAcelGen(vector<float> &acelT, vector<float> &acelN,  vector<float> &tan_x, vector<float> &tan_y, vector<float> &norm_x,
                    vector<float> &norm_y, vector<float> &acelgenX, vector<float> &acelgenY)
{

    for(int i=0; i<acelT.size(); i++)
    {

        acelgenX.push_back(acelT[i]*tan_x[i] +  acelN[i]*norm_x[i]);
        acelgenY.push_back(acelT[i]*tan_y[i] +  acelN[i]*norm_y[i]);

    }
}


void compute_Speed(vector<float> v_x, vector<float> v_y, vector<float> &feat_normVel)
{

    float val_velnorm_prev=0;
    for(int t=1; t<v_x.size(); t++)
    {
        float vecx_ = val_Distance1D(v_x[t], v_x[t-1]);
        float vecy_ = val_Distance1D(v_y[t], v_y[t-1]);
        float val_velnorm=0;
        if(val_euclnom(vecx_, vecy_)!= 0)
        {
            val_velnorm=val_euclnom(vecx_, vecy_);
        }
        else
        {
            val_velnorm=val_velnorm_prev;
        }
        feat_normVel.push_back(val_velnorm);
        val_velnorm_prev=val_velnorm;
    }

}

void compute_Velxy(vector<float> &v_x, vector<float> &v_y,  vector<float> &vec_vel_x, vector<float> &vec_vel_y, vector<float> &vec_vel_xS, vector<float> &vec_vel_yS)
{
    float val_velx_prev=0;
    float val_vely_prev=0;
    for(int t=1; t<v_x.size(); t++)
    {
        float vecx_ = val_Distance1D(v_x[t], v_x[t-1]);
        float vecy_ = val_Distance1D(v_y[t], v_y[t-1]);
        float val_velx=0;
        float val_vely=0;
        if(val_euclnom(vecx_, vecy_)!= 0)
        {
            val_velx=vecx_/val_euclnom(vecx_, vecy_);
            val_vely=vecy_/val_euclnom(vecx_, vecy_);
        }
        else
        {
            val_velx=val_velx_prev;
            val_vely=val_vely_prev;
        }
        vec_vel_x.push_back(val_velx);
        vec_vel_y.push_back(val_vely);
        vec_vel_xS.push_back(vecx_);
        vec_vel_yS.push_back(vecy_);
        val_velx_prev = val_velx;
        val_vely_prev = val_vely;

    }
}


void compute_Angle(vector<float> &vect_x, vector<float> &vect_y,  vector<float> &vectRet)
{
    for(int t=0; t<vect_x.size(); t++)
    {
        float angle = val_degree(vect_x[t], vect_y[t]);
        vectRet.push_back(angle);
    }
}


void compute_AnglebyBeans(vector<float> &vect_angle,  vector<float> &vectRet, int bines)
{

    for(int t=0; t<vect_angle.size(); t++)
    {
        float bean = (int)(vect_angle[t]*bines/360+ 0.5);
        vectRet.push_back(bean);
    }

}
