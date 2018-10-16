#ifndef BOKW_CODEBOOK_H
#define BOKW_CODEBOOK_H

#include <opencv2/opencv.hpp>

using namespace cv;


Mat* ToBuildCodeBook(vector<string> vec_val_train, vector<string> featLabels, vector<string> vec_activities, vector<float> alphaScales,
                     string str_path_trajectories, int randOption, int num_Words, int save_option, int &numScaleAllowed, string boWInfo);

Mat compDesPerScalePer(float*** sequenceKin,int frame_init, int frame_end, int num_tot_traj, int numTotalFrames, int size_featLab, float scale, int index_sf);
Mat compDesPerScaleCumulated(float*** sequenceKin,int frame_init, int frame_end, int frame_n,  int num_tot_traj, int numTotalFrames, int size_featLab, float scale, int index_sf);

Mat* CodeBooksPerAlpha(Mat** Mat_SamplesInputs, vector<float> alphaScales,
                       vector<string> vec_activities, int randOption, int num_Words, int &numScaleAllowed, int save_option, string boWInfo);

bool band_computeScale(Mat* Mat_SamplesInputs, int num_totalTrajec, int num_samples_perAct, int vec_activ_size);

Mat ToAddNewDescriptor(Mat tot_, Mat desc_, int size_featLab);
#endif // BOKW_CODEBOOK_H
