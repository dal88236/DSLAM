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

#ifndef SIMPLEGRAPH_H
#define SIMPLEGRAPH_H

#include <map>
#include <vector>
#include <utility>

namespace ORB_SLAM3
{

struct Node
{
    Node() {}
    Node(unsigned long id) : mnId(id) {}

    unsigned long mnId;
    Node* mpNext{nullptr};
    bool visited{false};
}; // struct Node

struct Vertex
{
    Vertex() {mnId = -1;}
    Vertex(unsigned long id) : mnId(id) {}
    unsigned long mnId;
}; // struct Vertex

struct Edge
{
    Edge() {mnSrcVId=-1; mnDestVId=-1;}
    Edge(unsigned long srcId, unsigned long destId) : mnSrcVId(srcId), mnDestVId(destId) {}
    unsigned long mnSrcVId;
    unsigned long mnDestVId;
}; // struct Edge

class Graph
{
public:
    Graph();
    Graph(const Graph &graph);
    ~Graph();

    void AddVertex(unsigned long inVertexId);
    bool HasVertex(unsigned long inVertexId);
    bool RemoveVertex(unsigned long inVertexId);
    void RemoveIsolatedVertices();
    void AddEdge(unsigned long inSrcVertexId, unsigned long inDestVertexId);
    bool RemoveEdge(unsigned long inSrcVertexId, unsigned long inDestVertexId);
    void Clear();
    int GetNumberOfVertices() const 
    {
        return mnVertices;
    }
    int GetNumberOfEdges() const 
    {
        return mnEdges;
    }
    void GetConnections(std::vector<std::vector<Vertex>>& vertexArray, std::vector<std::vector<Edge>>& edgeArray);
    std::vector<std::vector<unsigned long>> GetVertexConnections();
    void GetMaxConnections(std::vector<Vertex>& vertexArray, std::vector<Edge>& edgeArray);
    std::vector<unsigned long> GetMaxVertexConnection();

private:
    void SetAllNodesUnvisited();

    std::map<unsigned long, Node*> mmAdjList;
    int mnVertices;
    int mnEdges;
    bool mbTraversed{false};
}; // class Graph

} // namespace ORB_SLAM3

#endif // SIMPLEGRAPH_H