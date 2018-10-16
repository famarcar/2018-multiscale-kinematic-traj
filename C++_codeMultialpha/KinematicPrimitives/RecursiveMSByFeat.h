#ifndef RECURSIVEMSBYFEAT_H
#define RECURSIVEMSBYFEAT_H

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


using namespace cv;
using namespace std;




float computeRecursFeatByValue(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name, string recursive_name, float alpha);
vector<float> computeRecursFeatByVector(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name, string recursive_name, float alpha);


#endif // RECURSIVEMSBYFEAT_H
