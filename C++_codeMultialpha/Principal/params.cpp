#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>
#include <sstream>

using namespace std;




string root                       = "/home/fmartinezc/main/datasets/UT/"; //laptop
//string root                       = "/home/fmartinezc/Datasets/UT"; //oficce
//string root                       = "/home/fmartinezc/Datasets/KTH";

//string str_path_train_c           = root +  "/ConfigurationFiles/KTH_trainingSeq1.txt";
//string str_path_test_c            = root +  "/ConfigurationFiles/KTH_testSeq1.txt";

/// UT
string str_path_train_c           = root +  "/ConfigurationFiles/segmented_set1_rand.txt";
string str_path_test_c            = root +  "/ConfigurationFiles/segmented_set2_rand.txt";


string str_path_valid_c           = root +  "/ConfigurationFiles/KTH_validationSeq1.txt";
string str_path_activities_c      = root +  "/ConfigurationFiles/activities.txt";

string str_path_trajectories_c    = root +  "/improved/";
string str_path_videos_c          = root +  "/VIDEOS/";
string str_path_frames_c          = root +  "/FRAMES/";
string str_path_MaxMinKinFeat_c   = root +  "/ConfigurationFiles/MaxMinKinFeat.txt";


float scale_start_c               = 2.0;
int num_max_scales_c              = 3;// namely i  s 3

using namespace std;

vector<string> get_vect_features()
{
    vector<string> featLabels;
///----------------------------------------------------------------
//    featLabels.push_back("feat_devcurv;mean_f");
//    featLabels.push_back("feat_devcurv;maxmin_f");
////    //featLabels.push_back("feat_devcurv;std_f");
////
//    featLabels.push_back("feat_curv;mean_f");
//    featLabels.push_back("feat_curv;maxmin_f");
//////    featLabels.push_back("feat_curv;std_f");
////
///////----------------------------------------------------------------
////
//    featLabels.push_back("feat_acelT;mean_f");
//    featLabels.push_back("feat_acelT;maxmin_f");
//////    featLabels.push_back("feat_acelT;std_f");
//////
//    featLabels.push_back("feat_acelN;mean_f");
//    featLabels.push_back("feat_acelN;maxmin_f");
//////    featLabels.push_back("feat_acelN;std_f");
////
///////----------------------------------------------------------------
////
//    featLabels.push_back("feat_Tangy;mean_f");
//    featLabels.push_back("feat_Tangy;maxmin_f");
////    featLabels.push_back("feat_Tangy;std_f");
////
//    featLabels.push_back("feat_Tangx;mean_f");
//    featLabels.push_back("feat_Tangx;maxmin_f");
////    featLabels.push_back("feat_Tangx;std_f");
////
///////----------------------------------------------------------------
////
//    featLabels.push_back("feat_Normaly;mean_f");
//    featLabels.push_back("feat_Normaly;maxmin_f");
//////    featLabels.push_back("feat_Normaly;std_f");
//////
//    featLabels.push_back("feat_Normalx;mean_f");
//    featLabels.push_back("feat_Normalx;maxmin_f");    featLabels.push_back("feat_devcurv;mean_f");
//    featLabels.push_back("feat_devcurv;maxmin_f");
//    //featLabels.push_back("feat_devcurv;std_f");
//
    featLabels.push_back("feat_curv;mean_f");
    featLabels.push_back("feat_curv;maxmin_f");
////    featLabels.push_back("feat_curv;std_f");
//
/////----------------------------------------------------------------
//
    featLabels.push_back("feat_acelT;mean_f");
    featLabels.push_back("feat_acelT;maxmin_f");
////    featLabels.push_back("feat_acelT;std_f");
////
    //featLabels.push_back("feat_acelN;mean_f");
    //featLabels.push_back("feat_acelN;maxmin_f");
//////    featLabels.push_back("feat_acelN;std_f");
////
///////----------------------------------------------------------------
////
//    featLabels.push_back("feat_Tangy;mean_f");
//    featLabels.push_back("feat_Tangy;maxmin_f");
////    featLabels.push_back("feat_Tangy;std_f");
////
//    featLabels.push_back("feat_Tangx;mean_f");
//    featLabels.push_back("feat_Tangx;maxmin_f");
//    featLabels.push_back("feat_Tangx;std_f");
//
/////----------------------------------------------------------------
//
//    featLabels.push_back("feat_Normaly;mean_f");
//    featLabels.push_back("feat_Normaly;maxmin_f");
////    featLabels.push_back("feat_Normaly;std_f");
////
//    featLabels.push_back("feat_Normalx;mean_f");
//    featLabels.push_back("feat_Normalx;maxmin_f");
////    featLabels.push_back("feat_Normalx;std_f");
/////----------------------------------------------------------------
//
//      featLabels.push_back("feat_ThetaBin;mean_f");
//    featLabels.push_back("feat_ThetaBin;maxmin_f");
////    featLabels.push_back("feat_ThetaBin;std_f");
////
    featLabels.push_back("feat_theta;mean_f");
    featLabels.push_back("feat_theta;maxmin_f");
//    featLabels.push_back("feat_theta;std_f");
//
//      featLabels.push_back("feat_speed;mean_f");
//     featLabels.push_back("feat_speed;maxmin_f");
//    featLabels.push_back("feat_speed;std_f");

///----------------------------------------------------------------


    return featLabels;
}

vector<float> get_vect_scales()
{
    vector<float> alphaScales;
    for(int ni=1; ni <= num_max_scales_c; ni++)
    {
//        alphaScales.push_back((float)1.0/scale_start_c);
//        scale_start_c= scale_start_c*2; //s(t)=(s(t-1))*2
        alphaScales.push_back((float) pow(scale_start_c, -1*ni));

    }
    return alphaScales;
}

string get_path_train()
{
    return str_path_train_c;
}


string get_path_test()
{
    return str_path_test_c;
}

string get_path_valid()
{
    return str_path_valid_c;
}

string get_path_videos()
{
    return str_path_videos_c;
}

string get_path_trajectories()
{
    return str_path_trajectories_c;
}

string get_path_activities()
{
    return str_path_activities_c;
}

string get_path_frames()
{
    return str_path_frames_c;
}

string get_path_MaxMinKinFeat()
{
    return str_path_MaxMinKinFeat_c;
}
