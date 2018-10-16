/**---------------------------------------------------------------------------------------
Author: fabio.martinez-carillo@ensta-paristech.fr
Description: Methods to configurate the initial parameters of the strategy, namely the kineamtic features and the number of scales.
Date: 29/04/2015
----------------------------------------------------------------------------------------*/

#ifndef PARAMS_H
#define PARAMS_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;


string get_path_train();
string get_path_test();
string get_path_valid();
string get_path_videos();
string get_path_trajectories();
string get_path_activities();
string get_path_frames();
string get_path_expAct();
string get_path_MaxMinKinFeat();
vector<string> get_vect_features();
vector<float> get_vect_scales();
string get_path_HistFrameOneSeq();

#endif // PARAMS_H
