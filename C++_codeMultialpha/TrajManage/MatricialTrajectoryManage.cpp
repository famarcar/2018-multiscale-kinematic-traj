#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "../Resources/Matrixmanage.h"
#include "../Resources/GeneralFunctions.h"

#include "MatricialTrajectoryManage.h"

#include "../KinematicPrimitives/RecursiveMSByFeat.h"
#include "../KinematicPrimitives/OnlyKinematics.h"


using namespace std;





void ToLoadMatTraj(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                   int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTrajKin)
{
    //time sequence, totaloftraj, componentsOfTraj
    int tot_seq_matrix = (end_seq-star_seq)+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
    cout<< " num cuadros video: "<< tot_seq_matrix <<endl;
    ToInitMatrix3D(sequenceTrajKin, tot_seq_matrix, num_tot_traj, 3);
    int cont_tj=0;

    for(int iter_scales=0; iter_scales< num_scales_traj; iter_scales++)
    {
        // son escalas de matthieu method...no kinematic scales.
        stringstream ss_iscales;
        ss_iscales << iter_scales;
        string path_Traj = pathFileTrajectories + ss_iscales.str();
        //cout<< path_Traj<< endl;
        ifstream datafiles_stream(path_Traj.c_str(), fstream::in);
        string read_line;
        getline(datafiles_stream, read_line);//----- the first line to read the head of the file
        while(datafiles_stream.good())
        {
            string read_line;
            getline(datafiles_stream, read_line);
            if(read_line.size()>0)
            {
                vector<string> vt_pt_tj = splitStr(read_line,' ' );
                int init_traj = (atoi(vt_pt_tj[0].c_str())-1); // -1 para correrlas un punto
                int end_traj = (atoi(vt_pt_tj[1].c_str())-1);  // -1 para correrlas un punto

                //cout<< init_traj << " end "<< end_traj << endl;
                sequenceTrajKin[tot_seq_matrix-1][cont_tj][0] =  init_traj;
                sequenceTrajKin[tot_seq_matrix-1][cont_tj][1] =  end_traj;

                int cont_pt_tj=2;
                int cont_z=1;
                for(int m=init_traj; m < end_traj; m++)
                {
                    sequenceTrajKin[m][cont_tj][1] = atoi(vt_pt_tj[cont_pt_tj++].c_str());//y ---proablemente hay que cambiarlos de posicion porque vienen como y,x
                    sequenceTrajKin[m][cont_tj][0] = atoi(vt_pt_tj[cont_pt_tj++].c_str()); //x
                    sequenceTrajKin[m][cont_tj][2] = cont_z++;
                }
                //cout<< " cont_pt_tj: "<< cont_pt_tj << " vt_pt_tj:  "<< vt_pt_tj.size() << endl;
                cont_tj++;
            }
        }
    }


}



void ToLoadMatTrajOnly_XYZ(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                           int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTraj)
{



    //time sequence, totaloftraj, componentsOfTraj
    int tot_seq_matrix = (end_seq-star_seq)+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
    int cont_tj=0;

    for(int iter_scales=0; iter_scales< num_scales_traj; iter_scales++)
    {
        // son escalas de matthieu method...no kinematic scales.
        stringstream ss_iscales;
        ss_iscales << iter_scales;
        string path_Traj = pathFileTrajectories + ss_iscales.str();
        //cout<< path_Traj<< endl;
        ifstream datafiles_stream(path_Traj.c_str(), fstream::in);
        string read_line;
        getline(datafiles_stream, read_line);//----- the first line to read the head of the file
        while(datafiles_stream.good())
        {
            string read_line;
            getline(datafiles_stream, read_line);
            if(read_line.size()>0)
            {
                vector<string> vt_pt_tj = splitStr(read_line,' ' );
                int init_traj = (atoi(vt_pt_tj[0].c_str())-1); // -1 para correrlas un punto
                int end_traj = (atoi(vt_pt_tj[1].c_str())-1);  // -1 para correrlas un punto

                //cout<< init_traj << " end "<< end_traj << endl;
                sequenceTraj[tot_seq_matrix-1][cont_tj][0] =  init_traj;
                sequenceTraj[tot_seq_matrix-1][cont_tj][1] =  end_traj;

                int cont_pt_tj=2;
                int cont_z=1;
                for(int m=init_traj; m < end_traj; m++)
                {
                    sequenceTraj[m][cont_tj][1] = atof(vt_pt_tj[cont_pt_tj++].c_str());//y ---proablemente hay que cambiarlos de posicion porque vienen como y,x
                    sequenceTraj[m][cont_tj][0] = atof(vt_pt_tj[cont_pt_tj++].c_str()); //x
                    sequenceTraj[m][cont_tj][2] = cont_z;
                    cont_z++;
                }
                //cout<< " cont_pt_tj: "<< cont_pt_tj << " vt_pt_tj:  "<< vt_pt_tj.size() << endl;
                cont_tj++;
            }
        }
    }
}





void ToLoadMatTrajOnlyKin(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                          int width_frame, int num_tot_traj, int num_scales_traj, float*** sequenceTrajKin)
{



    //time sequence, totaloftraj, componentsOfTraj
    int tot_seq_matrix = (end_seq-star_seq)+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
    int cont_tj=0;

    for(int iter_scales=0; iter_scales< num_scales_traj; iter_scales++)
    {
        // son escalas de matthieu method...no kinematic scales.
        stringstream ss_iscales;
        ss_iscales << iter_scales;
        string path_Traj = pathFileTrajectories + ss_iscales.str();
        //cout<< path_Traj<< endl;
        ifstream datafiles_stream(path_Traj.c_str(), fstream::in);
        string read_line;
        getline(datafiles_stream, read_line);//----- the first line to read the head of the file
        while(datafiles_stream.good())
        {
            string read_line;
            getline(datafiles_stream, read_line);
            if(read_line.size()>0)
            {
                vector<string> vt_pt_tj = splitStr(read_line,' ' );
                int init_traj = (atoi(vt_pt_tj[0].c_str())-1); // -1 para correrlas un punto
                int end_traj = (atoi(vt_pt_tj[1].c_str())-1);  // -1 para correrlas un punto

                //cout<< init_traj << " end "<< end_traj << endl;
                sequenceTrajKin[tot_seq_matrix-1][cont_tj][0] =  init_traj;
                sequenceTrajKin[tot_seq_matrix-1][cont_tj][1] =  end_traj;

                int cont_pt_tj=2;
                int cont_z=1;
                for(int m=init_traj; m < end_traj; m++)
                {
                    sequenceTrajKin[m][cont_tj][1] = atof(vt_pt_tj[cont_pt_tj++].c_str()); //y ---proablemente hay que cambiarlos de posicion porque vienen como y,x
                    sequenceTrajKin[m][cont_tj][0] = atof(vt_pt_tj[cont_pt_tj++].c_str()); //x
                    sequenceTrajKin[m][cont_tj][2] = cont_z;
                    /// To add kinematics -------------------------------------
                    if(cont_z>2)
                    {
                        sequenceTrajKin[m][cont_tj][3]  = comp_Theta(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1]); //Theta
                        sequenceTrajKin[m][cont_tj][4]  = comp_Speed(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1]); //Speed
//
                        sequenceTrajKin[m][cont_tj][5]  = comp_Curv(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0], sequenceTrajKin[m-2][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1], sequenceTrajKin[m-2][cont_tj][1],
                                                          sequenceTrajKin[m][cont_tj][2], sequenceTrajKin[m-1][cont_tj][2], sequenceTrajKin[m-2][cont_tj][2]); //Curv

                        sequenceTrajKin[m][cont_tj][6]  = comp_Tangy(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1]);
                        sequenceTrajKin[m][cont_tj][7]  = comp_Tangx(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1]);
                        sequenceTrajKin[m][cont_tj][8]  = comp_Normy(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0], sequenceTrajKin[m-2][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1], sequenceTrajKin[m-2][cont_tj][1]);
                        sequenceTrajKin[m][cont_tj][9]  = comp_Normx(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0], sequenceTrajKin[m-2][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1], sequenceTrajKin[m-2][cont_tj][1]);
                        sequenceTrajKin[m][cont_tj][10] = comp_AcelT(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0], sequenceTrajKin[m-2][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1], sequenceTrajKin[m-2][cont_tj][1]);
                        sequenceTrajKin[m][cont_tj][11] = comp_AcelN(sequenceTrajKin[m][cont_tj][0], sequenceTrajKin[m-1][cont_tj][0], sequenceTrajKin[m-2][cont_tj][0],
                                                          sequenceTrajKin[m][cont_tj][1], sequenceTrajKin[m-1][cont_tj][1], sequenceTrajKin[m-2][cont_tj][1]);
                    }
                    ///---------------------------------------------------------


                    cont_z++;

                }
                //cout<< " cont_pt_tj: "<< cont_pt_tj << " vt_pt_tj:  "<< vt_pt_tj.size() << endl;
                cont_tj++;
            }
        }
    }



}

void ToLoadMatTrajKinRecFeat(int star_seq, int end_seq,string pathFileTrajectories, int height_frame,
                             int width_frame, int num_tot_traj,
                             vector<string> featLabels, vector<float> vect_scale, float*** sequenceTrajKin)
{
    int tot_seq_matrix = (end_seq-star_seq)+1; // mas dos porque en las dos ultimas posiciones se escribe el inicion y final de la trajectoria. Cabezera info.
    int tot_feat_matrix = ((int)featLabels.size()*(int)vect_scale.size())+3;

    int cont_tj=0;
    //scales of semi dense trajectories, there are not kinematic scales.


    string path_Traj = pathFileTrajectories;
    //cout<< path_Traj<< endl;
    ifstream datafiles_stream(path_Traj.c_str(), fstream::in);
    string read_line;
    getline(datafiles_stream, read_line);//----- the first line to read the head of the file
    while(datafiles_stream.good())
    {
        string read_line;
        getline(datafiles_stream, read_line);
        if(read_line.size()>0)
        {
            vector<string> vt_pt_tj = splitStr(read_line,' ' );
            int init_traj = atoi(vt_pt_tj[0].c_str())-1; // -1 para correrlas un punto
            int end_traj = atoi(vt_pt_tj[1].c_str())-1;  // -1 para correrlas un punto

            sequenceTrajKin[tot_seq_matrix-1][cont_tj][0] =  init_traj;
            sequenceTrajKin[tot_seq_matrix-1][cont_tj][1] =  end_traj;
            //----------------compute kin feat.....
            int sizeTraj = end_traj - init_traj;
            //cout <<" -----------------------sizeTraj: "<< sizeTraj << " ";
            if(sizeTraj>4) ///NAMELY IS 2 BUT TO WORK WITH DEV CURV
            {
                float** matrixKinFeat = ToCreateMatrix2D((sizeTraj-2), ((int)featLabels.size()*(int)vect_scale.size()));
                ToInitMatrix2D(matrixKinFeat, (sizeTraj-2), ((int)featLabels.size()*(int)vect_scale.size()));
                // cout<< " --------- (sizeTraj-2) -------" << (sizeTraj-2) << " sizeTraj "<< sizeTraj <<endl;
                int cont_feat_scale=0;
                for(int cont_scale=0; cont_scale<vect_scale.size(); cont_scale++)
                {
                    for(int cont_feat=0; cont_feat<featLabels.size(); cont_feat++ )
                    {
                        vector<string> v_staFeat = ToSepearatActionStatistic( featLabels[cont_feat]);
                        vector<float> pointFeat_x, pointFeat_y, pointFeat_z;
                        computeXYZvectors( vt_pt_tj, pointFeat_x, pointFeat_y, pointFeat_z);
                        vector<float> vectFeat = computeRecursFeatByVector( pointFeat_x, pointFeat_y, pointFeat_z,
                                                 v_staFeat[0],  v_staFeat[1], vect_scale[cont_scale]);

                        //cout<< "vectFeat " << v_staFeat[0]<< " vectFeat "<< vectFeat.size() << endl;
                        for(int l=0; l<vectFeat.size(); l++)
                        {
                            matrixKinFeat[l][cont_feat_scale]=vectFeat[l];


                        }

                        cont_feat_scale++;
                    }
                }
                //-------------------------------- Storage x, y points
                int cont_pt_tj=2;
                int cont_z=1;
                for(int m=init_traj; m < end_traj; m++)
                {
                    sequenceTrajKin[m][cont_tj][1] = atoi(vt_pt_tj[cont_pt_tj++].c_str());//y ---proablemente hay que cambiarlos de posicion porque vienen como y,x
                    sequenceTrajKin[m][cont_tj][0] = atoi(vt_pt_tj[cont_pt_tj++].c_str()); //x
                    sequenceTrajKin[m][cont_tj][2] = cont_z++;
                }
                //-------------------------------- Storage the kinematic features.
                int cont_kframe =0;
                for(int m=(init_traj+2); m<end_traj; m++)
                {
                    int cont_feat_scale=0;
                    for(int cont_scale=0; cont_scale<vect_scale.size(); cont_scale++)
                    {
                        for(int cont_feat=0; cont_feat<featLabels.size(); cont_feat++ )
                        {
                            sequenceTrajKin[m][cont_tj][(3+cont_feat_scale)] = matrixKinFeat[cont_kframe][cont_feat_scale];
                            cont_feat_scale++;
                        }
                    }
                    cont_kframe++;
                }
                //--------------------------------
                cont_tj++;
                ToEliminateMatrix2D(matrixKinFeat, (sizeTraj-2), (int)featLabels.size()*(int)vect_scale.size());
            }//else si uno quiere usar las trajectorias que miden menso que 4
        }
    }
    datafiles_stream.close();


}



//--------------------------------------------------------------------------------------------------------


void computeXYZvectors(vector<string> vt_pt_tj,
                       vector<float>  &pointFeat_x, vector<float>  &pointFeat_y,
                       vector<float>  &pointFeat_z)
{


    int init_traj = atoi(vt_pt_tj[0].c_str())-1; // -1 para correrlas un punto
    int end_traj = atoi(vt_pt_tj[1].c_str())-1;  // -1 para correrlas un punto
    int cont_pt_tj=2;
    for(int i=0; i<(end_traj-init_traj); i++)
    {

        pointFeat_y.push_back(atoi(vt_pt_tj[cont_pt_tj++].c_str()));//y ---proablemente hay que cambiarlos de posicion porque vienen como y,x
        pointFeat_x.push_back(atoi(vt_pt_tj[cont_pt_tj++].c_str())); //x
        pointFeat_z.push_back(i);

    }


}


/// modified by new version fmartinezc (26-12-2017)
void infoVideoFromTrajec(string pathFileTrajectories, int &num_frames, int &height_frame,
                         int &width_frame, int &num_tot_traj)
{





    ifstream datafiles_stream(pathFileTrajectories.c_str(), fstream::in);
    string read_line;
    getline(datafiles_stream, read_line);
    stringstream ss;
    vector<string> v;
    ss << read_line;
    string line;
    while (getline( ss, line, ' ' )) v.push_back( line );

    num_tot_traj = atoi(v[0].c_str());// number of trajectories
    height_frame = atoi(v[1].c_str());// y-axis
    width_frame  = atoi(v[2].c_str());// x-axis
    num_frames   = atoi(v[3].c_str());// number of frames
    datafiles_stream.close();

}

