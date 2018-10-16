#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "BoKW_Codebook.h"
#include "../Resources/Matrixmanage.h"
#include "../Resources/GeneralFunctions.h"

#include "../TrajManage/MatricialTrajectoryManage.h"

using namespace cv;
using namespace std;


/// Modified from original posdoc version by fmartinezc----
/// 26-12-2017
Mat* ToBuildCodeBook(vector<string> vec_val_train, vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales,
                     string str_path_trajectories, int randOption, int num_Words, int save_option, int &numScaleAllowed, string boWInfo)
{


    //structure to store the input samples organized by activity and scale parameter.
    cout << "alphaScales.size() : "<< alphaScales.size() << endl;
    Mat* Mat_CodeBooks = new Mat[alphaScales.size()];
    Mat** Mat_SamplesInputs  = ToCreateMatrixMat2D((int)alphaScales.size(), (int)vec_activities.size());



    for(int i=0; i<vec_val_train.size(); i++)
    {
        cout<< i <<") sequence "<< vec_val_train[i]<< endl;
        int numTotalFrames, heightFrame, widthFrame, num_tot_traj;

///KTH:        string path_textTrajectories = str_path_trajectories + vec_val_train[i] + ".txt"; //dense and improved
        string path_textTrajectories = str_path_trajectories + vec_val_train[i] + ".scale0"; //dense and improved

        cout<<" temporal test: "<< path_textTrajectories << endl;

        infoVideoFromTrajec(path_textTrajectories, numTotalFrames,
                                     heightFrame, widthFrame, num_tot_traj);

        cout<< i <<") sequence "<< vec_val_train[i] << " numFrames: "<< numTotalFrames << " numTrajec: "<< num_tot_traj  << endl;
        int tot_seq_matrix = numTotalFrames+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
        int tot_feat_matrix = ((int)featLabels.size()*(int)alphaScales.size())+3;

        cout<< "<----- creating mat: ["<< tot_seq_matrix<<"]["<< num_tot_traj<<"]["<< tot_feat_matrix<<"]    ------>"<<endl;
        float*** sequenceKin = ToCreateMatrix3D(tot_seq_matrix, num_tot_traj, tot_feat_matrix);
        ToInitMatrix3D(sequenceKin, tot_seq_matrix, num_tot_traj, tot_feat_matrix);

        ToLoadMatTrajKinRecFeat(0, numTotalFrames,path_textTrajectories,
                                heightFrame, widthFrame, num_tot_traj,
                                featLabels, alphaScales, sequenceKin);


        for( int nscales=0; nscales< alphaScales.size(); nscales++)
        {

            Mat desc_=compDesPerScalePer(sequenceKin, 0, numTotalFrames, num_tot_traj,  numTotalFrames, featLabels.size(), alphaScales[nscales], (nscales*featLabels.size()));
            //Mat_SamplesInputs[nscales][atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str())-1].push_back(desc_);
            Mat new_Mat = ToAddNewDescriptor(Mat_SamplesInputs[nscales][atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str())-1], desc_, featLabels.size());
            Mat_SamplesInputs[nscales][atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str())-1]= new_Mat;
            new_Mat.release();
            desc_.release();
       }
        ToEliminateMatrix3D(sequenceKin, tot_seq_matrix , num_tot_traj ,tot_feat_matrix);
    }


    Mat_CodeBooks = CodeBooksPerAlpha( Mat_SamplesInputs, alphaScales, vec_activities,
                                       randOption,  num_Words, numScaleAllowed, save_option,  boWInfo);

    ToEliminateMatrixMat2D(Mat_SamplesInputs, (int)alphaScales.size(), (int)vec_activities.size() );
    return Mat_CodeBooks;
}

Mat compDesPerScaleCumulated(float*** sequenceKin,int frame_init, int frame_end, int frame_n, int num_tot_traj,
                             int numTotalFrames, int size_featLab, float scale, int index_sf)
{
    Mat desc_seq;
    for(int traj_n=0; traj_n<num_tot_traj; traj_n++)  //num_tot_traj
    {
        int init_traj = sequenceKin[numTotalFrames][traj_n][0];
        int end_traj =  sequenceKin[numTotalFrames][traj_n][1];
        if(init_traj >= frame_init   && end_traj<frame_end)    //verificar si el los frames aca estan corridos.
        {

            int num_min_frame = (1/scale);
            if(sequenceKin[frame_n][traj_n][2]>num_min_frame)
            {
                Mat KF_traj = Mat::zeros(1, size_featLab, CV_32F);
                for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
                {
                    KF_traj.at<float>(0,feat_n) = sequenceKin[frame_n][traj_n][index_sf+feat_n+3];
                }
                desc_seq.push_back(KF_traj);
                KF_traj.release();
            }
        }

    }
    return desc_seq;
}



Mat compDesPerScalePer(float*** sequenceKin,int frame_init, int frame_end, int num_tot_traj, int numTotalFrames, int size_featLab, float scale, int index_sf)
{
    Mat desc_seq;
    for(int traj_n=0; traj_n<num_tot_traj; traj_n++)  //num_tot_traj
    {
        int init_traj = sequenceKin[frame_end][traj_n][0];
        int end_traj =  sequenceKin[frame_end][traj_n][1];

        int sizeTraj = end_traj - init_traj;
        //if(sizeTraj>4) ///NAMELY IS 2 BUT TO WORK WITH DEV CURV
        //{
            int num_min_frame = (1/scale) + init_traj;
            for(int frame_n = num_min_frame; frame_n< end_traj; frame_n++) // este frame se cambia para que deje de ser denso.
                //se puede cambiar en lugar de endtraj colocar num_min_frame
            {
                Mat KF_traj = Mat::zeros(1, size_featLab, CV_32F);

                for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
                {
                    KF_traj.at<float>(0,feat_n) = sequenceKin[frame_n][traj_n][index_sf+feat_n+3];
                }

                desc_seq.push_back(KF_traj);
                KF_traj.release();
            }
        //}

    }
    return desc_seq;
}

Mat ToAddNewDescriptor(Mat tot_, Mat desc_, int size_featLab )
{

    Mat newMat;

    if(tot_.rows >0)
    {

        newMat = Mat::zeros((int)(tot_.rows+desc_.rows), size_featLab, CV_32F);

        int contGlobalrows=0;

        for(int row_n =0; row_n< tot_.rows; row_n++)
        {
            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
            {
                newMat.at<float>(contGlobalrows,feat_n) = tot_.at<float>(row_n,feat_n);
            }
            contGlobalrows++;
        }


        for(int row_n =0; row_n< desc_.rows; row_n++)
        {
            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
            {
                newMat.at<float>(contGlobalrows,feat_n) = desc_.at<float>(row_n,feat_n);
            }
            contGlobalrows++;
        }

    }
    else
    {

        newMat = Mat::zeros((int)(desc_.rows), size_featLab, CV_32F);
        for(int row_n =0; row_n< desc_.rows; row_n++){
            for(int feat_n= 0; feat_n<size_featLab; feat_n++ )
            {
                newMat.at<float>(row_n,feat_n) = desc_.at<float>(row_n,feat_n);
            }
            }
    }




return newMat;

}

Mat* CodeBooksPerAlpha(Mat** Mat_SamplesInputs, vector<float> alphaScales,
                       vector<string> vec_activities, int randOption, int num_Words, int &numScaleAllowed, int save_option, string boWInfo)
{

    Mat* Mat_dictionaries = new Mat[alphaScales.size()];


    ofstream outdataBoW_(boWInfo.c_str(), fstream::out);
    cout<< "..... building alpha vocabularies.... "<< endl;
    TermCriteria tc(CV_TERMCRIT_ITER,200,0.001);
    int retries =3; //retries number
    int flags   =KMEANS_PP_CENTERS;//necessary flags
    int num_k   = 0;
///-------------------------------------------------------------------

    for(int alpha_n=0; alpha_n< alphaScales.size(); alpha_n++)
    {
        Mat vec_perAlpha, ilabelsAlpha; // store all samples per alpha
        cout << " scale: "<< alpha_n << endl;
        int num_totalTrajec = numTotal_Traj(Mat_SamplesInputs[alpha_n], vec_activities.size());
        cout<< " num_totalTrajec: "<<num_totalTrajec<< endl;
        int num_samples_perAct= (num_totalTrajec/vec_activities.size())*0.1;
        bool band_scale = band_computeScale(Mat_SamplesInputs[alpha_n], num_totalTrajec, num_samples_perAct, vec_activities.size());

        outdataBoW_ <<"--------------------------" << endl;
        outdataBoW_ << " num_samples_perAct " << num_samples_perAct << " num_totalSamples "<<num_totalTrajec
                    <<" scale value: 1/"<< 1/alphaScales[alpha_n]<<endl;
        outdataBoW_ <<"--------------------------" << endl;
//----------------------------------------------------------------------------------------------------------------------
        if(band_scale == true)
        {
            numScaleAllowed++;
            outdataBoW_ <<"-------------------------- Num Scale: "  <<numScaleAllowed  << " Scale: "<< alpha_n << endl;
            for(int act_n=0; act_n< vec_activities.size(); act_n++)
            {
                //---------random selection of sample per activity -------------------
                Mat featuresforVocabulary_selected;
                if(randOption==1)
                {
                    RNG rng( 0xFFFFFFFF );
                    outdataBoW_<< " @@@ vec_perActivity.rows "<< Mat_SamplesInputs[alpha_n][act_n].rows << " num_samples_perAct " << num_samples_perAct << endl;
                    for(int i_ran=0; i_ran<  num_samples_perAct; i_ran++)
                    {
                        featuresforVocabulary_selected.push_back(Mat_SamplesInputs[alpha_n][act_n].row(rng.uniform( 0, Mat_SamplesInputs[alpha_n][act_n].rows )));
                    }
                }
                else
                {
                    featuresforVocabulary_selected = Mat_SamplesInputs[alpha_n][act_n];
                }

                vec_perAlpha.push_back(featuresforVocabulary_selected);
                outdataBoW_<< "&&&&& Per Activity: " << act_n <<" "<<featuresforVocabulary_selected.rows << " cols "<< featuresforVocabulary_selected.cols <<endl;
                featuresforVocabulary_selected.release();
//------------------------------------------------------------------------------------------------------------------------------------

            }

            outdataBoW_<< "Con Alpha: 1/"<< 1/alphaScales[alpha_n] <<" : "<<vec_perAlpha.rows << " cols "<< vec_perAlpha.cols <<endl;
            cout<< "Con Alpha: 1/"<< 1/alphaScales[alpha_n] <<" : "<<vec_perAlpha.rows << " cols "<< vec_perAlpha.cols <<endl;
            //Mat_dictionaries[alpha_n] = KMeansComputationPerDic(vec_perAlpha,0, 0 ); //400
            if(num_Words==0)
            {
                num_k = sqrt(vec_perAlpha.rows/4); //namely /2
                //num_k = log2(vec_perAlpha.rows)*vec_perAlpha.cols; //namely /2
            }
            else
            {
                num_k = num_Words;
            }
            outdataBoW_<< " num_Words: "<< num_k << " or equal to: "<< sqrt(vec_perAlpha.rows/4)/2 << " from a total of:  "<< vec_perAlpha.rows << endl;
            double result = kmeans(vec_perAlpha, num_k, ilabelsAlpha, tc, retries, flags,Mat_dictionaries[alpha_n]); //400
            vec_perAlpha.release();
            ilabelsAlpha.release();
        }
    }
    outdataBoW_<< "num scales allowed: "<< numScaleAllowed << endl;

    for(int itDic=0; itDic < alphaScales.size(); itDic++)
    {
        outdataBoW_<<"itDic: "<< itDic <<"dictionary_ap1: rows " <<  (Mat_dictionaries[itDic]).rows << " cols "<<  (Mat_dictionaries[itDic]).cols <<endl;
        cout<<"itDic: "<< itDic <<"dictionary_ap1: rows " <<  (Mat_dictionaries[itDic]).rows << " cols "<<  (Mat_dictionaries[itDic]).cols <<endl;
        if(save_option==1)
        {
            stringstream itDic_str;
            itDic_str<<itDic;
            FileStorage fs(("dict_"+ itDic_str.str() +".yml"), FileStorage::WRITE);
            fs << "vocabulary" << Mat_dictionaries[itDic];
            fs.release();
        }
    }
    return Mat_dictionaries;

}

bool band_computeScale(Mat* Mat_SamplesInputs, int num_totalTrajec, int num_samples_perAct, int vec_activ_size)
{
    bool bandera =false;
    int cont=0;

    for(int act_n=0; act_n< vec_activ_size; act_n++)
    {
        int num_per_activity=0;
        num_per_activity = num_per_activity + Mat_SamplesInputs[act_n].rows;//-> Mat
        if(num_per_activity > num_samples_perAct)
        {
            cont++;
        }
    }
    if(cont==vec_activ_size)
    {
        bandera= true;
    }
    return bandera;

}

