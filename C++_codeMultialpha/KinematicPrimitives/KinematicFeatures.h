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


float computeKinFeatByValue(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name);
vector<float> computeKinFeatByVector(vector<float> vect_traj_x, vector<float> vect_traj_y, vector<float> vect_traj_z, string feature_name);





