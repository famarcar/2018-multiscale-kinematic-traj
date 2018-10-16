/**---------------------------------------------------------------------------------------
Author: fabio.martinez-carillo@ensta-paristech.fr
Description: Methods to built histograms from a BoW approach. This cpp include recirsive and global histograms for namely video-sequences.
Date: 01/06/2015
----------------------------------------------------------------------------------------*/
#ifndef BOWHISTMATRIX_H
#define BOWHISTMATRIX_H

void computeCumulHist(Mat* Codebooks, string nameFileOut, string str_path_trajectories, vector<string> vec_val_train,
                      vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales,  int numScaleAllowed, int cumulated, int num_scalesTrajectories);

void computeFrameLevelHist(Mat* Codebooks, string nameFileOut, string str_path_trajectories, vector<string> vec_val_train,
                           vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales, int numScaleAllowed,   int cumulated, int num_scalesTrajectories);



float ** histogramSequencePerFrame( float*** sequenceKin, Mat* Codebooks, int frame_init, int frame_end, int numScaleAllowed,
                                    vector<string> featLabels, vector<float> alphaScales, int numTotalFrames, int num_tot_traj);

float ** HistogramSequenceComplete( float*** sequenceKin, Mat* Codebooks, int frame_init, int frame_end, int numScaleAllowed,
                                    vector<string> featLabels, vector<float> alphaScales, int numTotalFrames, int num_tot_traj);

void computeCumulHistOneSequence(Mat* Codebooks, string nameFileOut, string str_path_trajectories,
                                 string str_val_train,
                                 vector<string> featLabels, vector<string> vec_activities,
                                 vector<float> alphaScales, int numScaleAllowed,   int cumulated);

#endif // BOWHISTMATRIX_H
