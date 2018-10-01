# 2018-multiscale-kinematic-traj

## Multi-temporal kinematic characterization of semi-dense trajectories for action recognition

### Fabio Martínez,  Antoine Manzanera, Michèle Gouiffès



This work introduces a multi-temporal scale motion descriptor to classify and recognize activities by characterizing the kinematics of trajectories in a video sequence. The method starts by extracting (semi)-dense kinematics from video-motion trajectories, which represent relevant and statistically significant motion cues.  At each time, every trajectory is characterized by a set of kinematic statistics that code the ongoing motion activity in a relative appearance-independent way. These features are recursively estimated for different time scales, corresponding to different lengths of the trajectories. They form motion-cue words that are mapped to a set of previously built codebooks, and a frame-level activity histogram is computed. The proposed descriptor is able at each frame, allowing on-line action recognition capability. The proposed approach is evaluated using a bag-of-features framework, in which every frame-level histogram is mapped to a SVM. Three different types of trajectories were tested. The proposed approach was tested for complete action sequences in different datasets: KTH , UT-interaction, and UCF-Youtube showing a competitive performance in terms of accuracy and computation time. Also, the strategy was evaluated for on-line action recognition, in which the activity is predicted at each frame of the video sequences. In such evaluation the strategy achieved a online recognition of $\sim 85.51 \% $ by using only the $60 \%$ of the video sequence.
