/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STATIC_POINT_DETERMINATION_H
#define STATIC_POINT_DETERMINATION_H

#include <queue>
#include <mutex>

#include "KeyFrame.h"
#include "Atlas.h"

namespace ORB_SLAM3
{

class StaticPointDetermination
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StaticPointDetermination(Atlas* pAtlas, size_t th=10);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

     // Thread Synch
    void RequestStop();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void SetNumOfIterations(int iterations);

    void InterruptSPD();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

protected:
    bool CheckNewKeyFrames();
    bool CheckAllKeyFramesProcessed();
    void ProcessNewKeyFrame();

    void MapPointCulling();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpUnmarkedMapPoints;

    std::mutex mMutexNewKFs;

    std::list<KeyFrame*> mlNewKeyFrames;
    std::list<KeyFrame*> mlKeyFramesInSW;

    bool mbAbortOptimization;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    int mnNewKF;
    int mnProcessedKF;

    int mnIterations;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    size_t mnQueueTh;
}; // class StaticPointDetermination

} // namespace ORB_SLAM3

#endif // STATIC_POINT_DETERMINATION_H