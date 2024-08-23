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

#ifndef DELAUNAY_TRIANGULATION_2D_H
#define DELAUNAY_TRIANGULATION_2D_H

#include <opencv2/core.hpp>

namespace ORB_SLAM3
{

struct Triangle {
    cv::Point2f mVertices[3];
}; // struct Triangle

struct Circle {
    cv::Point2f mCenter;
    float mRadius;
}; // struct Circle

class DelaunayTriangulation2DBuilder
{
public:
    DelaunayTriangulation2DBuilder();

private:
    // reorder the points in ascending order
    void Reorder(std::vector<cv::Point>& vP);
    Circle GetCircumscribedCircle(const Triangle& tri);
}; // class DelaunayTriangulation2DBuilder

} // namespace ORB_SLAM3

#endif // DELAUNAY_TRIANGULATION_2D_H