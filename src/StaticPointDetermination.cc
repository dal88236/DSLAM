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

#include "StaticPointDetermination.h"
#include "Optimizer.h"

#include <mutex>
#include <chrono>

namespace ORB_SLAM3
{

StaticPointDetermination::StaticPointDetermination(Atlas* pAtlas, size_t nQueueTh):
    mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mbAbortOptimization(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mnLastProcessedKF(-1),
    mbAcceptKeyFrames(true), mnQueueTh(nQueueTh)
{

}

void StaticPointDetermination::Run()
{
    mbFinished = false;

    while(1)
    {
        SetAcceptKeyFrames(false);

        if(!CheckAllKeyFramesProcessed())
        {
            ProcessNewKeyFrame();
            bool bDoneSPD = false;

            mbAbortOptimization = false;
            
            if(!stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {
                    KeyFrame* pFirstKF = mlNewKeyFrames.front();
                    Map* pCurrentMap = pFirstKF->GetMap();
                    Optimizer::LocalPointGraphOptimization(mlNewKeyFrames, &mbAbortOptimization, pCurrentMap, mnIterations, mlpUnmarkedMapPoints);
                }
                bDoneSPD = true;
                mnLastProcessedKF = mlNewKeyFrames.back()->mnId;
            }
        }

        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void StaticPointDetermination::MapPointCulling(unsigned long nKFId)
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpUnmarkedMapPoints.begin();

    while(lit!=mlpUnmarkedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if(!pMP)
            lit = mlpUnmarkedMapPoints.erase(lit);
        else if(pMP->isBad())
            lit = mlpUnmarkedMapPoints.erase(lit);
        else if(!pMP->isMarked())
        {
            if(pMP->mnLastSeenKF <= nKFId)
            {
                pMP->SetBadFlag();
                lit = mlpUnmarkedMapPoints.erase(lit);
            }
        }

        lit++;
    }
}

void StaticPointDetermination::InsertKeyFrame(KeyFrame *pKF)
{
    if(!AcceptKeyFrames())
        return;
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);

    while(mlNewKeyFrames.size() > mnQueueTh)
    {
        KeyFrame* pKF = mlNewKeyFrames.front();
        pKF->ClearDepthMat();
        mlNewKeyFrames.pop_front();
        MapPointCulling(pKF->mnId);
    }
}

void StaticPointDetermination::RemoveKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.remove(pKF);
}

bool StaticPointDetermination::CheckAllKeyFramesProcessed()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    if(mlNewKeyFrames.empty())
        return true;

    return mnLastProcessedKF < 0? false : mlNewKeyFrames.back()->mnId<=mnLastProcessedKF;
}

void StaticPointDetermination::ProcessNewKeyFrame()
{
    unique_lock<mutex> lock(mMutexNewKFs);
        
    for(list<KeyFrame*>::iterator lit=mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;

        if(pKF->mnId <= mnLastProcessedKF)
            continue;

        vector<MapPoint*> vpMapPointMatches = pKF->GetMapPointMatches();
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                pMP->mnLastSeenKF = pKF->mnId;
                if(!pMP->isMarked() && !pMP->mbProcessedBySPD) // Dynamic points detected in front end
                {
                    mlpUnmarkedMapPoints.push_back(pMP);
                    pMP->mbProcessedBySPD = true;
                }
            }
        }
    }
    
}

void StaticPointDetermination::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortOptimization = true;
}

bool StaticPointDetermination::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Static Point Determination STOP" << endl;
        return true;
    }

    return false;
}

bool StaticPointDetermination::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool StaticPointDetermination::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void StaticPointDetermination::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Static Point Determination RELEASE" << endl;
}

bool StaticPointDetermination::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void StaticPointDetermination::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool StaticPointDetermination::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void StaticPointDetermination::SetNumOfIterations(int iterations)
{
    mnIterations = iterations;
}

void StaticPointDetermination::InterruptSPD()
{
    mbAbortOptimization = true;
}

void StaticPointDetermination::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool StaticPointDetermination::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void StaticPointDetermination::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool StaticPointDetermination::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void StaticPointDetermination::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in SPD..." << endl;
            mlNewKeyFrames.clear();
            mlpUnmarkedMapPoints.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;

            cout << "LM: End reseting SPD..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in SPD..." << endl;
            mlNewKeyFrames.clear();
            mlpUnmarkedMapPoints.clear();

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting SPD..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

} // namespace ORB_SLAM3