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

#include "DelaunayTriangulation2d.h"

#include <algorithm>

namespace ORB_SLAM3
{

// TODO
DelaunayTriangulation2DBuilder::DelaunayTriangulation2DBuilder() {}

void DelaunayTriangulation2DBuilder::Reorder(std::vector<cv::Point>& vP) 
{
    std::sort(vP.begin(), vP.end(), [](cv::Point P1, cv::Point P2) -> bool {
        return P1.x < P2.x;
    });
}

Circle DelaunayTriangulation2DBuilder::GetCircumscribedCircle(const Triangle& tri) 
{
    Circle circumScribedCircle;
    cv::Point2f midPoint1((tri.mVertices[0].x + tri.mVertices[1].x) * 0.5, (tri.mVertices[0].y + tri.mVertices[1].y) * 0.5);
    cv::Point2f midPoint2((tri.mVertices[1].x + tri.mVertices[2].x) * 0.5, (tri.mVertices[1].y + tri.mVertices[2].y) * 0.5);
    float m1 = 1 / (tri.mVertices[1].y - tri.mVertices[0].y) / (tri.mVertices[1].x - tri.mVertices[0].x);
    float m2 = 1 / (tri.mVertices[2].y - tri.mVertices[1].y) / (tri.mVertices[2].x - tri.mVertices[1].x);
    float c1 = midPoint1.y - m1 * midPoint1.x;
    float c2 = midPoint2.y * m2 * midPoint2.x;
    circumScribedCircle.mCenter.x = (c2 - c1) / (m1 - m2);
    circumScribedCircle.mCenter.y = m1 * circumScribedCircle.mCenter.x + c1;
    circumScribedCircle.mRadius = std::sqrt((circumScribedCircle.mCenter.x - tri.mVertices[0].x) * (circumScribedCircle.mCenter.x - tri.mVertices[0].x)+
                                            (circumScribedCircle.mCenter.y - tri.mVertices[0].y) * (circumScribedCircle.mCenter.y - tri.mVertices[0].y));
    return circumScribedCircle;
}
} // namespace ORB_SLAM3