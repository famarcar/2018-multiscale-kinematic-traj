#include <iostream>
#include <math.h>

#include "OnlyKinematics.h"
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

float val_eucldist(float val_ax, float val_ay)
{
    return sqrt(pow((val_ax),2) + pow((val_ay),2) );
}

float euclDistance3D(float val_ax, float val_ay, float val_az,  float val_bx, float val_by,  float val_bz)
{
    return sqrt(pow((val_ax-val_bx),2) + pow((val_ay-val_by),2)+ pow((val_az-val_bz),2));
}

float comp_Speed(float x_act, float x_prev, float y_act, float y_prev)
{

    /// ||v(t)||

    float x_comp = x_act - x_prev;
    float y_comp = y_act - y_prev;

    return val_eucldist(y_comp, x_comp);
}


float comp_Theta(float x_act, float x_prev, float y_act, float y_prev)
{

    float x_comp = x_act - x_prev;
    float y_comp = y_act - y_prev;

    return fastAtan2(y_comp, x_comp);
}

float eval_curv(float a, float b, float c, float s)
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

float comp_Curv(float x_act, float x_prev1, float x_prev2,
                float y_act, float y_prev1, float y_prev2,
                float t_act, float t_prev1, float t_prev2)
{

    float a = euclDistance3D(x_prev1, y_prev1, t_prev1, x_prev2, y_prev2, t_prev2);
    float b = euclDistance3D(x_act  , y_act,     t_act, x_prev1, y_prev1, t_prev1);
    float c = euclDistance3D(x_act  , y_act,     t_act, x_prev2, y_prev2, t_prev2);
    float s = (a+b+c)/2;

    return eval_curv(a, b, c, s);

}

float comp_Tangy(float x_act, float x_prev1, float y_act, float y_prev1)
{
    /// Ty = vy(t)/||v(t)||

    float x_comp = x_act - x_prev1;
    float y_comp = y_act - y_prev1;
    float norm = val_eucldist(y_comp, x_comp);
    if(norm == 0)
    {
        return 1;
    }
    else
    {
        return y_comp/norm;
    }

}

float comp_Tangx(float x_act, float x_prev1, float y_act, float y_prev1)
{
    /// Tx = vx(t)/||v(t)||


    float x_comp = x_act - x_prev1;
    float y_comp = y_act - y_prev1;
    float norm = val_eucldist(y_comp, x_comp);

    if(norm == 0)
    {
        return 1;
    }
    else
    {
        return x_comp/norm;
    }


}

float comp_Normy(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2)
{
    ///N(t) = T'(t)/||T'(t)||

    float Tany_act  =  comp_Tangy( x_act,  x_prev1,  y_act,  y_prev1);
    float Tany_prev =  comp_Tangy( x_prev1, x_prev2,  y_prev1,  y_prev2);

    float dev_Ty        = Tany_act - Tany_prev;
    float nor_dev_Ty    = val_eucldist(Tany_act, Tany_prev);

    if(nor_dev_Ty == 0)
    {
        return 1;
    }
    else
    {
        return dev_Ty/nor_dev_Ty;
    }

}

float comp_Normx(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2)
{
    ///N(t) = T'(t)/||T'(t)||

    float Tanx_act  =  comp_Tangx( x_act,  x_prev1,  y_act,  y_prev1);
    float Tanx_prev =  comp_Tangx( x_prev1, x_prev2,  y_prev1,  y_prev2);

    float dev_Tx        = Tanx_act - Tanx_prev;
    float nor_dev_Tx    = val_eucldist(Tanx_act, Tanx_prev);

    if(nor_dev_Tx == 0)
    {
        return 1;
    }
    else
    {
        return dev_Tx/nor_dev_Tx;
    }

}

float comp_AcelT(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2)
{
    /// at = d(||v(t)||)/dt
    float at_act  = comp_Speed( x_act,  x_prev1,  y_act,  y_prev1);
    float at_prev = comp_Speed( x_prev1,  x_prev2,  y_prev1,  y_prev2);

    return (at_act-at_prev);


}

float comp_AcelN(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2)
{
    /// an = ||v(t)||.||T'(t)||
    float at_act  = comp_Speed( x_act,  x_prev1,  y_act,  y_prev1);
    float Tanx_act  =  comp_Tangx( x_act,  x_prev1,  y_act,  y_prev1);
    float Tanx_prev =  comp_Tangx( x_prev1, x_prev2,  y_prev1,  y_prev2);
    float nor_dev_Tx    = val_eucldist(Tanx_act, Tanx_prev);

    return (at_act*nor_dev_Tx);
}

