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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp> // esto es para que funcione bruteforce

#include "BoKW_Codebook.h"
#include "BoWHistMatrix.h"

#include "../Resources/Matrixmanage.h"
#include "../Resources/GeneralFunctions.h"

#include "../TrajManage/MatricialTrajectoryManage.h"

using namespace cv;
using namespace std;


/// Modified from original posdoc version by fmartinezc----
/// 24-01-2018


void computeCumulHist(Mat* Codebooks, string nameFileOut, string str_path_trajectories, vector<string> vec_val_train,
                      vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales, int numScaleAllowed,   int cumulated, int num_scalesTrajectories)
{


    string strForNormal= nameFileOut + ".txt";  //vector noramlizado per each bin
    ofstream outdata_(strForNormal.c_str(), fstream::out);
    outdata_.precision(15);
    outdata_<<  fixed;

    string strForhistoTotal = nameFileOut +"_vtn" + ".txt"; //vector total noramlizado
    ofstream outdataHistTot_(strForhistoTotal.c_str(), fstream::out);
    outdataHistTot_.precision(15);
    outdataHistTot_<<  fixed;

    for(int i=0; i<vec_val_train.size(); i++)
    {

        cout<< i <<") sequence "<< vec_val_train[i]  << endl;
        int numTotalFrames, heightFrame, widthFrame, num_tot_traj;

        string path_textTrajectories = str_path_trajectories + vec_val_train[i] + ".txt"; //dense and improved

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



//aca voy
//-----------------------------------------------------------------------------------------------------------------------------


        outdata_ << atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str()) << " ";
        outdataHistTot_<< atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str()) << " ";

        float **vect_HistGlobal;

        if(cumulated==1)
        {
            vect_HistGlobal = histogramSequencePerFrame( sequenceKin, Codebooks,  0,  numTotalFrames,  numScaleAllowed,
                              featLabels,  alphaScales,  numTotalFrames, num_tot_traj);
        }
        else
        {
            vect_HistGlobal  = HistogramSequenceComplete( sequenceKin, Codebooks,  0,  numTotalFrames,  numScaleAllowed,
                               featLabels,  alphaScales,  numTotalFrames, num_tot_traj);
        }

        ///------------------------ starting with writing process  -------------------------------------------------
        // store the max value for each scale to normalize per scale.
        float *max_valuesPerScale = ToCreateMatrix1D(numScaleAllowed);
        ToInitMatrix1D(max_valuesPerScale, numScaleAllowed);
        float max_val=0;
        for(int i=0; i<numScaleAllowed; i++)
        {
            for(int j=0; j<Codebooks[i].rows; j++)
            {
                max_valuesPerScale[i] = max_valuesPerScale[i] + vect_HistGlobal[i][j];
                max_val = max_val + vect_HistGlobal[i][j];
            }
        }
        int cont = 0;
        for(int i=0; i<numScaleAllowed; i++)
        {
            for(int j=0; j<Codebooks[i].rows; j++)
            {
                outdataHistTot_<< (cont+1) << ":"<< vect_HistGlobal[i][j]/max_val <<" ";
                outdata_       << (cont+1) << ":"<< vect_HistGlobal[i][j]/max_valuesPerScale[i] <<" ";
                cont++;
            }
        }
        cout<<" escribio ok subsequence to file "<< endl;
        outdata_<<endl;
        outdataHistTot_<< endl;


        ToEliminateMatrix1D(max_valuesPerScale, numScaleAllowed);
        ToEliminateMatrix2D(vect_HistGlobal, numScaleAllowed, numScaleAllowed);



        ///-------------------------------------------------------------------------------------------------------


        ToEliminateMatrix3D(sequenceKin, (numTotalFrames+1) , num_tot_traj ,(((int)featLabels.size()*(int)alphaScales.size())+3));

    }

    outdata_.close();
    outdataHistTot_.close();
}




///-------------------------------- Implemented  2/04/2018 -------------------------

/// Modified from original posdoc version by fmartinezc----
/// 24-01-2018


void computeFrameLevelHist(Mat* Codebooks, string nameFileOut, string str_path_trajectories, vector<string> vec_val_train,
                           vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales, int numScaleAllowed,   int cumulated, int num_scalesTrajectories)
{

    ofstream outdata_array[11];
    ofstream outdataHistTot_array[11];


    int val_write =10;
    for(int zz=0; zz<11; zz++)
    {


        stringstream ss_number;
        ss_number << val_write;
        string strForhistoTotal = nameFileOut +"vtn_" + ss_number.str() + ".txt";
        string strForNormal     = nameFileOut  + ss_number.str() + ".txt";

        outdataHistTot_array[zz].open(strForhistoTotal.c_str(), fstream::out);
        outdata_array[zz].open(strForNormal.c_str(), fstream::out);

        outdata_array[zz].precision(15);
        outdata_array[zz]<< fixed;
        outdataHistTot_array[zz].precision(15);
        outdataHistTot_array[zz]<< fixed;
        val_write = val_write +10;
    }


    for(int i=0; i<vec_val_train.size(); i++)
    {
        cout<< i <<") sequence "<< vec_val_train[i]  << endl;
        int numTotalFrames, heightFrame, widthFrame, num_tot_traj;
        string path_textTrajectories = str_path_trajectories + vec_val_train[i] + ".txt";


        infoVideoFromTrajec(path_textTrajectories, numTotalFrames, heightFrame, widthFrame, num_tot_traj);

        cout<< i <<") sequence "<< vec_val_train[i] << " numFrames: "<< numTotalFrames << " numTrajec: "<< num_tot_traj  << endl;


        int tot_seq_matrix = numTotalFrames+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
        int tot_feat_matrix = ((int)featLabels.size()*(int)alphaScales.size())+3;


        cout<< "<----- creating mat: ["<< tot_seq_matrix<<"]["<< num_tot_traj<<"]["<< tot_feat_matrix<<"]    ------>"<<endl;
        float*** sequenceKin = ToCreateMatrix3D(tot_seq_matrix, num_tot_traj, tot_feat_matrix);
        ToInitMatrix3D(sequenceKin, tot_seq_matrix, num_tot_traj, tot_feat_matrix);

        ToLoadMatTrajKinRecFeat(0, numTotalFrames,path_textTrajectories,
                                heightFrame, widthFrame, num_tot_traj,
                                featLabels, alphaScales, sequenceKin);


        for(int zz=0; zz<11; zz++)
        {
            outdata_array[zz]<< atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str()) << " ";
            outdataHistTot_array[zz]<< atoi(returnLabelAction_KTH(vec_val_train[i], vec_activities).c_str()) << " ";
        }



        /// IMPORTANT: TO INCLUDE
        /// histogramPerFrameONLINE

        ///---------------------------------------------------------------------------------------------------------
        ///----------------------------- histogramPerFrameONLINE ---------------------------------------------------

        int frame_n =0;
        int video_percentaje= 10;
        for(int z=0; z<=10; z++)  // recuperar 9 segmentos del video y el total
        {
            frame_n = frame_n + floor(numTotalFrames/10);
            cout<< "z: "<< z << "- porcentaje : "<< video_percentaje << " frame_n : "<< frame_n   << endl;




            ///***********************************************************************************
            float **vect_HistGlobal;
            if(video_percentaje==110)
            {
                vect_HistGlobal  = histogramSequencePerFrame( sequenceKin, Codebooks,  0,  numTotalFrames,  numScaleAllowed,
                                   featLabels,  alphaScales,  numTotalFrames, num_tot_traj);

            }
            else
            {

                vect_HistGlobal  = histogramSequencePerFrame( sequenceKin, Codebooks,  0,  frame_n,  numScaleAllowed,
                                   featLabels,  alphaScales,  numTotalFrames, num_tot_traj);

            }


            ///***********************************************************************************



            /// normalizacion de histos.
            float *max_valuesPerScale = ToCreateMatrix1D(numScaleAllowed);
            ToInitMatrix1D(max_valuesPerScale, numScaleAllowed);
            float max_val=0;
            for(int i=0; i<numScaleAllowed; i++)
            {
                for(int j=0; j<Codebooks[i].rows; j++)
                {
                    max_valuesPerScale[i] = max_valuesPerScale[i] + vect_HistGlobal[i][j];
                    max_val = max_val + vect_HistGlobal[i][j];
                }
            }


            int cont=0;


            for(int i=0; i<numScaleAllowed; i++)
            {
                for(int j=0; j<Codebooks[i].rows; j++)
                {
                    float val_hist_ps = 0;
                    float val_hist_g =  0;

                    if(max_valuesPerScale[i]>0){

                       val_hist_ps = vect_HistGlobal[i][j]/max_valuesPerScale[i];
                    }
                    if(max_val>0){
                        val_hist_g = vect_HistGlobal[i][j]/max_val;
                    }


                    outdata_array[z]       << (cont+1) << ":"<< val_hist_ps <<" ";

                    outdataHistTot_array[z]<< (cont+1) << ":"<< val_hist_g <<" ";
                    cont++;

                }// end if(video_percentaje ==10)

            }


            outdata_array[z]<<endl;
            outdataHistTot_array[z]<< endl;

            video_percentaje = video_percentaje+10;
            ToEliminateMatrix1D(max_valuesPerScale, numScaleAllowed);
            ToEliminateMatrix2D(vect_HistGlobal, numScaleAllowed, numScaleAllowed);
        }

        ToEliminateMatrix3D(sequenceKin, (numTotalFrames+1) , num_tot_traj ,(((int)featLabels.size()*(int)alphaScales.size())+3));
    }



    for(int zz=0; zz<11; zz++)
    {
        outdata_array[zz].close();
        outdataHistTot_array[zz].close();
    }


}



///----------------------------------------------------------------------------------


float ** histogramPerFrameONLINE( float*** sequenceKin, Mat* Codebooks, int frame_init, int frame_end, int numScaleAllowed,
                                  vector<string> featLabels, vector<float> alphaScales, int numTotalFrames, int num_tot_traj)
{
    float **vect_HistGlobal = new float*[numScaleAllowed];
    int bines_totales = 0;
    for(int i=0; i<numScaleAllowed; i++)
    {
        vect_HistGlobal[i] = new float[Codebooks[i].rows];
        bines_totales = bines_totales + Codebooks[i].rows;
    }
    for(int i=0; i<numScaleAllowed; i++)
    {
        for(int j=0; j<Codebooks[i].rows; j++)
        {
            vect_HistGlobal[i][j]=0;
        }
    }

    for(int frame_n=frame_init; frame_n<frame_end; frame_n++)
    {
        for(int itDic = 0; itDic< numScaleAllowed; itDic++)
        {

            Mat desc_=compDesPerScaleCumulated(sequenceKin, frame_init, frame_end, frame_n, num_tot_traj,  numTotalFrames,
                                               featLabels.size(), alphaScales[itDic], (itDic*featLabels.size()));
            int num_bines = Codebooks[itDic].rows;
            Ptr<DescriptorMatcher> matcherT(new BFMatcher(cv::NORM_L2));
            matcherT->clear();
            vector<DMatch> matches;

            if(Codebooks[itDic].rows>1 && desc_.rows >1 )
            {
                matcherT->match( desc_, Codebooks[itDic], matches );
                for( int i = 0; i < matches.size(); i++ )
                {
                    vect_HistGlobal[itDic][matches[i].trainIdx] = vect_HistGlobal[itDic][matches[i].trainIdx] +1.f;
                }
            }


        }
    }

    return vect_HistGlobal;

}




///--------------------------------------------------------------------------------------
float ** histogramSequencePerFrame( float*** sequenceKin, Mat* Codebooks, int frame_init, int frame_end, int numScaleAllowed,
                                    vector<string> featLabels, vector<float> alphaScales, int numTotalFrames, int num_tot_traj)
{
    float **vect_HistGlobal = new float*[numScaleAllowed];
    int bines_totales = 0;
    for(int i=0; i<numScaleAllowed; i++)
    {
        vect_HistGlobal[i] = new float[Codebooks[i].rows];
        bines_totales = bines_totales + Codebooks[i].rows;
    }
    for(int i=0; i<numScaleAllowed; i++)
    {
        for(int j=0; j<Codebooks[i].rows; j++)
        {
            vect_HistGlobal[i][j]=0;
        }
    }

    for(int frame_n=frame_init; frame_n<frame_end; frame_n++)
    {
        for(int itDic = 0; itDic< numScaleAllowed; itDic++)
        {

            Mat desc_=compDesPerScaleCumulated(sequenceKin, frame_init, frame_end, frame_n, num_tot_traj,  numTotalFrames,
                                               featLabels.size(), alphaScales[itDic], (itDic*featLabels.size()));
            int num_bines = Codebooks[itDic].rows;
            Ptr<DescriptorMatcher> matcherT(new BFMatcher(cv::NORM_L2));
            matcherT->clear();
            vector<DMatch> matches;

            if(Codebooks[itDic].rows>1 && desc_.rows >1 )
            {
                matcherT->match( desc_, Codebooks[itDic], matches );
                for( int i = 0; i < matches.size(); i++ )
                {
                    vect_HistGlobal[itDic][matches[i].trainIdx] = vect_HistGlobal[itDic][matches[i].trainIdx] +1.f;
                }
            }


        }
    }

    return vect_HistGlobal;

}


float ** HistogramSequenceComplete( float*** sequenceKin, Mat* Codebooks, int frame_init, int frame_end, int numScaleAllowed,
                                    vector<string> featLabels, vector<float> alphaScales, int numTotalFrames, int num_tot_traj)
{
    float **vect_HistGlobal = new float*[numScaleAllowed];
    int bines_totales = 0;
    for(int i=0; i<numScaleAllowed; i++)
    {
        vect_HistGlobal[i] = new float[Codebooks[i].rows];
        bines_totales = bines_totales + Codebooks[i].rows;
    }
    for(int i=0; i<numScaleAllowed; i++)
    {
        for(int j=0; j<Codebooks[i].rows; j++)
        {
            vect_HistGlobal[i][j]=0;
        }
    }

    for(int itDic = 0; itDic< numScaleAllowed; itDic++)
    {
        Mat desc_=compDesPerScalePer(sequenceKin, frame_init, frame_end, num_tot_traj,  numTotalFrames, featLabels.size(), alphaScales[itDic], (itDic*featLabels.size()));
        int num_bines = Codebooks[itDic].rows;
        Ptr<DescriptorMatcher> matcherT(new BFMatcher(cv::NORM_L2));
        matcherT->clear();
        vector<DMatch> matches;

        if(Codebooks[itDic].rows>1 && desc_.rows >1 )
        {
            matcherT->match( desc_, Codebooks[itDic], matches );
            for( int i = 0; i < matches.size(); i++ )
            {
                vect_HistGlobal[itDic][matches[i].trainIdx] = vect_HistGlobal[itDic][matches[i].trainIdx] +1.f;
            }
        }

    }


    return vect_HistGlobal;

}

