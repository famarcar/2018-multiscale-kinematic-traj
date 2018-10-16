#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>



#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


/*
Funtion splitStr: To read a string and return a vector<string> with a split segments of the string.
                  The split criteriaum is carried out according to a criterium pass as a char.
*/
vector<string> splitStr(string str, char delimiter)
{
    vector<string> internal;
    stringstream ss(str); // Turn the string into a stream.
    string tok;

    while(getline(ss, tok, delimiter))
    {
        internal.push_back(tok);
    }

    return internal;
}


/*
Funtion redFileSequences: To read a file text and store in a vector<string> the lines of the text
                          the file is pointed according to a path passed as a string.
*/
void redFileSequences(string file_name, vector<string> &vec_Training)
{

    ifstream datafiles_stream(file_name.c_str(), fstream::in);
    while(datafiles_stream.good())
    {
        string read_line;
        getline(datafiles_stream, read_line);
        if(read_line.size()>0)
            vec_Training.push_back(read_line);
    }
}

/*
ToSepearatActionStatistic: Each kineatic feaure is codified as "kinfeat;statistic". Then
                           the method separate this two informations to compute a particular
                           Kinematic w.r.t a specific statistic.
*/
vector<string> ToSepearatActionStatistic(string featLabel_i)
{
    vector<string> v_staFeat;
    stringstream ss_stratFeat;
    string line2;
    ss_stratFeat <<featLabel_i;
    while(getline( ss_stratFeat, line2, ';' )) v_staFeat.push_back( line2 );
    return v_staFeat;

}


/*
returnLabelAction_UT: Given the name of the video it is returned the action number.
*/
string returnLabelAction_UT(string VideoName, vector<string> vec_activities )
{
    string act_return;
    stringstream ss_label;
    vector<string> v_label;
    string line2;
    ss_label << VideoName;
    while (getline( ss_label, line2, '_' )) v_label.push_back( line2 );
    for(int i=0; i<vec_activities.size(); i++)
    {
        if(v_label[2].compare(vec_activities[i])        == 0)
        {
            stringstream ss_activitynumber;
            ss_activitynumber << (i+1);
            act_return = ss_activitynumber.str();
        }
    }
    return act_return;

}


/*
returnLabelAction_KTH: Given the name of the video it is returned the action number.
*/
string returnNameLabelAction_KTH(string VideoName, vector<string> vec_activities )
{
    string act_return;
    stringstream ss_label;
    vector<string> v_label;
    string line2;
    ss_label << VideoName;
    while (getline( ss_label, line2, '_' )) v_label.push_back( line2 );
//    for(int i=0; i<vec_activities.size(); i++)
//    {
//        if(v_label[1].compare(vec_activities[i])        == 0)
//        {
//            stringstream ss_activitynumber;
//            ss_activitynumber << (i+1);
//            act_return = ss_activitynumber.str();
//        }
//    }
    return v_label[1];
}



/*
returnLabelAction_KTH: Given the name of the video it is returned the action number.
*/
string returnLabelAction_KTH(string VideoName, vector<string> vec_activities )
{
    string act_return;
    stringstream ss_label;
    vector<string> v_label;
    string line2;
    ss_label << VideoName;
    while (getline( ss_label, line2, '_' )) v_label.push_back( line2 );
    for(int i=0; i<vec_activities.size(); i++)
    {
        if(v_label[1].compare(vec_activities[i])        == 0)
        {
            stringstream ss_activitynumber;
            ss_activitynumber << (i+1);
            act_return = ss_activitynumber.str();
        }
    }
    return act_return;
}


/*
return label for the diferent gestures evaluated in SgL
*/
string returnLabelAction_SgL_date(string VideoName, vector<string> vec_activities )
{



    vector<string> vectSplitAct = splitStr(VideoName, '_');
    string vid_nameComp = vectSplitAct[2];

    for(int j= 3; j<(vectSplitAct.size()-1); j++)
    {
        vid_nameComp = vid_nameComp + "_" + vectSplitAct[j];
    }


    for(int i=0; i<vec_activities.size(); i++)
    {
        vector<string> vectSplitAct = splitStr(vec_activities[i], '-');
        if(vectSplitAct[0].compare(vid_nameComp ) == 0)
        {
            return vectSplitAct[1];
        }


    }



}

string returnLabelAction_SgL_days(string VideoName, vector<string> vec_activities )
{

    vector<string> vectSplitAct = splitStr(VideoName, '_');
    string vid_nameComp = vectSplitAct[2];

    for(int i=0; i<vec_activities.size(); i++)
    {
        vector<string> vectSplitAct = splitStr(vec_activities[i], '-');
        if(vectSplitAct[0].compare(vid_nameComp ) == 0)
        {
            return vectSplitAct[1];
        }

    }

}


string returnLabelAction_SgL_months(string VideoName, vector<string> vec_activities )
{

    vector<string> vectSplitActM = splitStr(VideoName, '_');
    string vid_nameComp = vectSplitActM[4];

    string strReturn = "0";

    for(int i=0; i<vec_activities.size(); i++)
    {
        vector<string> vectSplitAct = splitStr(vec_activities[i], '-');
        if(vectSplitAct[0].compare(vid_nameComp ) == 0
                &&
                vectSplitActM[vectSplitActM.size() -1].compare(2,1,"0") == 0
          )
        {
            strReturn= vectSplitAct[1];
        }

    }

    return strReturn;

}





/*
scale_: Scale a data between 0 and 255 to drawn in an image.
*/

float scale_( float y_min, float y_max, float value_in )
{
    float y_lower=0, y_upper=255;
    float value=0;

    if(value_in == y_min)
    {
        value = y_lower;
    }
    else if(value_in == y_max)
    {
        value = y_upper;
    }
    else
    {
        value = (value_in - y_min) * (y_upper - y_lower) / (y_max - y_min) + y_lower; //  para que acepte negative values


        //value = y_lower + (y_upper-y_lower) *(value_in - y_min)/(y_max-y_min);
    }

    return value;
}


int numTotal_Traj(Mat* Mat_SamplesInputs, int vec_activ_size)
{
    int num_tot=0;
    for(int act_n=0; act_n< vec_activ_size; act_n++)
    {
        num_tot = num_tot + Mat_SamplesInputs[act_n].rows;//-> Mat

    }
    return num_tot;
}


/// comment: 26-12-2017 by fmartinezc
//Mat ToAddNewDescriptor(Mat tot_, Mat desc_, int size_featLab )
//{
//
//    Mat newMat;
//
//    if(tot_.rows >0)
//    {
//
//        newMat = Mat::zeros((int)(tot_.rows+desc_.rows), size_featLab, CV_32F);
//
//        int contGlobalrows=0;
//
//        for(int row_n =0; row_n< tot_.rows; row_n++)
//        {
//            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
//            {
//                newMat.at<float>(contGlobalrows,feat_n) = tot_.at<float>(row_n,feat_n);
//            }
//            contGlobalrows++;
//        }
//
//
//        for(int row_n =0; row_n< desc_.rows; row_n++)
//        {
//            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
//            {
//                newMat.at<float>(contGlobalrows,feat_n) = desc_.at<float>(row_n,feat_n);
//            }
//            contGlobalrows++;
//        }
//
//    }
//    else
//    {
//
//        newMat = Mat::zeros((int)(desc_.rows), size_featLab, CV_32F);
//        for(int row_n =0; row_n< desc_.rows; row_n++)
//        {
//            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
//            {
//                newMat.at<float>(row_n,feat_n) = desc_.at<float>(row_n,feat_n);
//            }
//        }
//    }
//
//
//    return newMat;
//
//}


// take into account the trajectories with history superior to two.
int num_traj_valides_kin(float** sequenceKin_k, int num_tot_traj_k)
{

    int cont=0;
    for(int j=0; j<num_tot_traj_k; j++)
        if(sequenceKin_k[j][2]>2) cont++;

    return  cont;
}


