#ifndef ONLYKINEMATICS_H
#define ONLYKINEMATICS_H


float val_eucldist(float val_ax, float val_ay);

float euclDistance3D(float val_ax, float val_ay, float val_az,  float val_bx, float val_by,  float val_bz);

float comp_Speed(float x_act, float x_prev, float y_act, float y_prev);


float comp_Theta(float x_act, float x_prev, float y_act, float y_prev);

float eval_curv(float a, float b, float c, float s);

float comp_Curv(float x_act, float x_prev1, float x_prev2,
                float y_act, float y_prev1, float y_prev2,
                float t_act, float t_prev1, float t_prev2);

float comp_Tangy(float x_act, float x_prev1, float y_act, float y_prev1);

float comp_Tangx(float x_act, float x_prev1, float y_act, float y_prev1);

float comp_Normy(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2);

float comp_Normx(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2);

float comp_AcelT(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2);

float comp_AcelN(float x_act, float x_prev1, float x_prev2,
                 float y_act, float y_prev1, float y_prev2);


#endif // ONLYKINEMATICS_H
