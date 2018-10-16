/**---------------------------------------------------------------------------------------
Author: fabio.martinez-carillo@ensta-paristech.fr
Description: Methods to manage the trajectory information of a vide-seuqence by using a matricial way
             In this case it is easy to acces to the spatio-temporal information and perform
             faster computations.
Date: 22/05/2015
----------------------------------------------------------------------------------------*/

#ifndef MATRICIALTRAJECTORYMANAGE_H
#define MATRICIALTRAJECTORYMANAGE_H



void ToLoadMatTraj(int star_seq, int end_seq, string pathFileTrajectories, int height_frame,
                   int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTrajKin);

void ToLoadMatTrajOnlyKin(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                          int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTrajKin);

/// modified by new version fmartinezc (26-12-2017)
void ToLoadMatTrajKinRecFeat(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                             int width_frame, int num_tot_traj, vector<string> featLabels, vector<float> vect_scale, float*** sequenceTrajKin);

void computeXYZvectors(vector<string> vt_pt_tj,
                       vector<float>  &pointFeat_x, vector<float>  &pointFeat_y,
                       vector<float>  &pointFeat_z);

/// modified by new version fmartinezc (26-12-2017)
void infoVideoFromTrajec(string pathFileTrajectories, int &num_frames, int &height_frame,
                         int &width_frame, int &num_tot_traj);


void ToLoadMatTrajKinMoGFeat(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                             int width_frame, int num_tot_traj, int num_scales_traj,
                             vector<string> featLabels, int NumGauss, float*** sequenceTrajKin,
                             float alphaMoG, float std_init, float w_init, float rho_init, int th_MoG);

void ToLoadMatTrajOnly_XYZ(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                           int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTraj);



/// new method to codify also wang features and kinematic features
// void ToLoadMatTrajKinFeatWangFeat()
// void ToLoadMarTrajOnlyWangFeat()

#endif // MATRICIALTRAJECTORYMANAGE_H
