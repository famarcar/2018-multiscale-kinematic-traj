/**---------------------------------------------------------------------------------------
Author: fabio.martinez-carillo@ensta-paristech.fr
Description: Herein there some useful functions about manage of data that can be used for any othe function.
Date: 25/05/2015
----------------------------------------------------------------------------------------*/

#ifndef GENERALFUNCTIONS_H
#define GENERALFUNCTIONS_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

vector<string> splitStr(string str, char delimiter);
void redFileSequences(string file_name, vector<string> &vec_Training);
vector<string> ToSepearatActionStatistic(string featLabel_i);

string returnLabelAction_KTH(string VideoName, vector<string> vec_activities );
string returnNameLabelAction_KTH(string VideoName, vector<string> vec_activities );

string returnLabelAction_UT(string VideoName, vector<string> vec_activities );

float scale_( float y_min, float y_max, float value_in );
int numTotal_Traj(Mat* Mat_SamplesInputs, int vec_activ_size);
Mat ToAddNewDescriptor(Mat tot_, Mat desc_, int size_featLab );


string returnLabelAction_SgL_date(string VideoName, vector<string> vec_activities );
string returnLabelAction_SgL_days(string VideoName, vector<string> vec_activities );
string returnLabelAction_SgL_months(string VideoName, vector<string> vec_activities );

int num_traj_valides_kin(float** sequenceKin_k, int num_tot_traj_k);
bool symPosM_label(float* EigenValSym, int NFEATURES);
#endif // GENERALFUNCTIONS_H
