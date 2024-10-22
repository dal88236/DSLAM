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

#include "GraphBuilder.h"
#include "Frame.h"
#include "System.h"

#include <utility>

namespace ORB_SLAM3
{

GraphBuilder::GraphBuilder() {}

Delaunay GraphBuilder::BuildGraph(Frame &CurrentFrame, Frame &LastFrame)
{
    // examine
    std::vector<std::pair<Point, unsigned long>> dtPointsInput;
    for(int i=0; i<CurrentFrame.N; i++)
    {
        MapPoint* pMP = CurrentFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(pMP->isBad() || !pMP->isMarked()) continue;

            unsigned long nMPId = pMP->mnId;
            if(!LastFrame.mhMapPointsIDIdx.count(nMPId)) continue;

            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            dtPointsInput.emplace_back(std::make_pair(Point(x3Dw.x(), x3Dw.y(), x3Dw.z()), nMPId));
        }
    }
    Delaunay dt(dtPointsInput.begin(), dtPointsInput.end());
    Verbose::PrintMess("Constructed a Delaunay graph with " + to_string(dt.number_of_vertices()) +
     " vertices and " + to_string(dt.number_of_finite_edges()) + " edges", Verbose::VERBOSITY_DEBUG);

    return dt;
}

Delaunay GraphBuilder::BuildGraph(const std::list<MapPoint*>& lLocalMapPoints)
{
    std::vector<std::pair<Point, unsigned long>> dtPointsInput;
    for(std::list<MapPoint*>::const_iterator it=lLocalMapPoints.begin(), iend=lLocalMapPoints.end(); it!=iend; it++)
    {
        MapPoint* pMP = *it;
        if(pMP)
        {
            if(pMP->isBad() || !pMP->isMarked()) continue;

            unsigned long nMPId = pMP->mnId;

            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            dtPointsInput.emplace_back(std::make_pair(Point(x3Dw.x(), x3Dw.y(), x3Dw.z()), nMPId));
        }
    }
    Delaunay dt(dtPointsInput.begin(), dtPointsInput.end());
    Verbose::PrintMess("Local SPD: Constructed a graph with " + to_string(dt.number_of_vertices()) +
     " vertices and " + to_string(dt.number_of_finite_edges()) + " edges", Verbose::VERBOSITY_DEBUG);

    return dt;
}

} // namespace ORB_SLAM3 