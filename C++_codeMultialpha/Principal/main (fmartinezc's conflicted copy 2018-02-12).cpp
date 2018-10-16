/**
author: Fabio M.
       famarcar@uis.edu.co
Goal: To repeat experiment of global multhi-alpha characterization by using multiple alphas.
      Once a general score is achieved we pass to develop several experiments with per frame accuracy
      Experiment 1. Compute wang descriptor and test the performance
      Experiment 2. To test multi-alpha values
      Experiment 3. To test both experiments.

      Regarding the online prediction, The idea is to plot the accuracy of the approach by predicting at each frame.

      Experiment 1: By using a global dictionarry from the general approach
      Experiment 2: By training a per-frame dictionary and the compute the per-frame predictions
      Experiment 3: To add the other dense trajectory descriptors by Huang

      Finally to submit in a github project
*/


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
#include <time.h>

#include "params.h"


#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp> // esto es para que funcione bruteforce


#include "../BoWK_Building/BoKW_Codebook.h"
#include "../BoWK_Building/BoWHistMatrix.h"
#include  "../Resources/GeneralFunctions.h"


using namespace std;
using namespace cv;


///============ Global variables ======================

string str_path_train, str_path_test, str_path_valid, str_path_videos, str_path_trajectories,
       str_path_activities, str_path_frames, str_path_MaxMinKinFeat;
string str_path_expAct;

vector<string>  vec_test, vec_val_train,
       featLabels, vec_activities;



vector<float> alphaScales;


Mat* Codebooks; //n codebooks, one for each scale.

///============= funtion definition ===================
void ToInitparam();
void ToLoadFiles();
//void redFileSequences_main(string file_name, vector<string> &vec_Training);


int main(int argc, char *argv[])
{
    ToInitparam();
    ToLoadFiles();

    cout<< "Starting experiment with: "<< vec_val_train.size() << " vec_val_train and " <<
        vec_test.size() << " vec_test videos" << endl;
    /*----------------------------------
       1) To built the dictionary... each N*frames to compute the kinematic feature
    /----------------------------------*/

    int randOption=1, num_Words=0, save_option=1, numScaleAllowed=0;
    Codebooks = ToBuildCodeBook(vec_val_train,  featLabels, vec_activities, alphaScales,
                                str_path_trajectories,  randOption, num_Words, save_option,
                                numScaleAllowed, "BoWInfo_SqrtDiv2_Th4");

    /*----------------------------------
       2) To buil histograms per-frame: cumulative histogram
                                        global histograms
    /----------------------------------*/

        int cumulated =1; //1=cumulated; 0=completed
        int num_scalesTrajectories=0;

        computeCumulHist( Codebooks, "train_SqrtDiv2_Th4", str_path_trajectories, vec_val_train,
                                         featLabels, vec_activities, alphaScales,  numScaleAllowed, cumulated, num_scalesTrajectories );

        computeCumulHist( Codebooks, "test_SqrtDiv2_Th4", str_path_trajectories, vec_test,
                                         featLabels, vec_activities , alphaScales,  numScaleAllowed, cumulated, num_scalesTrajectories);



    /*----------------------------------
       3) To buil histograms per-frame and generate descriptos at each frame
    /----------------------------------*/

        /// famarcar@saber.uis.edu.co ---
        /// compute at each frame a descriptor. The output file has 100 descriptors
        computePerFrameHist( Codebooks, "test_SqrtDiv2_Th4", str_path_trajectories, vec_test,
                                         featLabels, vec_activities , alphaScales,  numScaleAllowed, cumulated, num_scalesTrajectories);

    cout << "Termino todo ok!" << endl;
    return 0;
}


void ToInitparam()
{

    str_path_train          = get_path_train();
    str_path_test           = get_path_test();
    str_path_valid          = get_path_valid();
    str_path_videos         = get_path_videos();
    str_path_trajectories   = get_path_trajectories();
    str_path_activities     = get_path_activities();
    str_path_frames         = get_path_frames();
    str_path_MaxMinKinFeat  = get_path_MaxMinKinFeat();
    alphaScales             = get_vect_scales();
    featLabels              = get_vect_features();
}

void ToLoadFiles()
{
    redFileSequences(str_path_test, vec_test);
    redFileSequences(str_path_train, vec_val_train);
    redFileSequences(str_path_valid, vec_val_train);
    redFileSequences(str_path_activities, vec_activities);

}


