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

#include "SimpleGraph.h"

#include <stack>
#include <algorithm>
#include <unordered_set>
#include <iostream>

namespace ORB_SLAM3
{

Graph::Graph() : mnVertices(0), mnEdges(0)
{
    
}

Graph::Graph(const Graph &graph)
    :mnVertices(graph.mnVertices), mnEdges(graph.mnEdges), mmAdjList(graph.mmAdjList)
{

}

Graph::~Graph()
{
    Clear();
}

void Graph::AddVertex(unsigned long inVertexId)
{
    if(mmAdjList.count(inVertexId))
        return;
    Node* node = new Node();
    node->mnId = inVertexId;
    mmAdjList[inVertexId] = node;
    ++mnVertices;
}

bool Graph::HasVertex(unsigned long inVertexId)
{
    return mmAdjList.count(inVertexId);
}

bool Graph::RemoveVertex(unsigned long inVertexId)
{
    if(!mmAdjList.count(inVertexId))
        return false;

    Node* node = mmAdjList[inVertexId];
    mmAdjList.erase(inVertexId);
    while(node!=nullptr)
    {
        Node* temp = node->mpNext;
        delete node;
        node = temp;
    }

    return true;
}

void Graph::RemoveIsolatedVertices()
{
    for(std::map<unsigned long, Node*>::const_iterator it=mmAdjList.begin(), iend=mmAdjList.end(); it!=iend;)
    {
        if(it->second->mpNext == nullptr)
        {
            Node* node = it->second;
            it = mmAdjList.erase(it); 
            delete node;
            --mnVertices;
        }
        else
            ++it;
    }
}

void Graph::AddEdge(unsigned long inSrcVertexId, unsigned long inDestVertexId)
{
    if(!mmAdjList.count(inSrcVertexId) || !mmAdjList.count(inDestVertexId))
    {
        std::cerr << "No corresponding vertices found!" << std::endl;
        return;
    }

    Node* nodeSrc = mmAdjList[inSrcVertexId];
    Node* nodeDest = mmAdjList[inDestVertexId];

    Node* newSrcAdjNode = new Node(inDestVertexId);
    newSrcAdjNode->mpNext = nodeSrc->mpNext;
    nodeSrc->mpNext = newSrcAdjNode;
    
    Node* newDestAdjNode = new Node(inSrcVertexId);
    newDestAdjNode->mpNext = nodeDest->mpNext;
    nodeDest->mpNext = newDestAdjNode;

    ++mnEdges;
}

bool Graph::RemoveEdge(unsigned long inSrcVertexId, unsigned long inDestVertexId)
{
    if(!mmAdjList.count(inSrcVertexId) || !mmAdjList.count(inDestVertexId))
        return false;

    Node* nodeSrc = mmAdjList[inSrcVertexId];
    Node* prevAdjSrcNode = nodeSrc;
    Node* adjSrcNode = nodeSrc->mpNext;

    if(adjSrcNode==nullptr)
        return false;

    bool deletedSrcEdge = false;
    while(adjSrcNode!=nullptr)
    {
        if(adjSrcNode->mnId==inDestVertexId)
        {
            prevAdjSrcNode->mpNext = adjSrcNode->mpNext;
            deletedSrcEdge = true;
            delete adjSrcNode;
            break;
        }
        prevAdjSrcNode = adjSrcNode;
        adjSrcNode = adjSrcNode->mpNext;
    }

    Node* nodeDest = mmAdjList[inDestVertexId];
    Node* prevAdjDestNode = nodeDest;
    Node* adjDestNode = nodeDest->mpNext;

    if(adjDestNode==nullptr)
        return false;

    bool deletedDestEdge = false;
    while(adjDestNode!=nullptr)
    {
        if(adjDestNode->mnId==inSrcVertexId)
        {
            prevAdjDestNode->mpNext = adjDestNode->mpNext;
            deletedDestEdge = true;
            delete adjDestNode;
            break;
        }
        prevAdjDestNode = adjDestNode;
        adjDestNode = adjDestNode->mpNext;
    }

    mnEdges--;

    return deletedSrcEdge && deletedDestEdge;
}

void Graph::Clear()
{
    for(auto iter=mmAdjList.begin(); iter!=mmAdjList.end(); iter++)
    {
        Node* node = iter->second->mpNext;
        while(node!=nullptr)
        {
            Node* temp = node->mpNext;
            delete node;
            node = temp;
        }
        delete iter->second;
    }
    mmAdjList.clear();
    mnVertices = 0;
    mnEdges = 0;
}

void Graph::GetConnections(std::vector<std::vector<Vertex>>& vertexArray, std::vector<std::vector<Edge>>& edgeArray)
{
    int count = 0;
    std::unordered_set<unsigned long> visited;
    for(std::map<unsigned long, Node*>::const_iterator it=mmAdjList.begin(), iend=mmAdjList.end(); it!=iend; it++)
    {
        if(visited.count(it->first)) 
            continue;

        std::stack<unsigned long> sNode;
        sNode.push(it->first);
        std::vector<Vertex> vertexConnection;
        std::vector<Edge> edgeConnection;
        vertexConnection.reserve(mnVertices);
        edgeConnection.reserve(mnEdges);

        while(!sNode.empty())
        {
            unsigned long nextId = sNode.top();
            sNode.pop();

            if(visited.count(nextId))
                continue;
            visited.insert(nextId);
            vertexConnection.push_back({nextId});

            Node* node = mmAdjList[nextId];
            Node* neighborNode = node->mpNext;
            while(neighborNode!=nullptr)
            {
                if(visited.count(neighborNode->mnId)) 
                {
                    neighborNode = neighborNode->mpNext;
                    continue;
                }

                if(node->mnId<neighborNode->mnId)
                {
                    // vertexConnection.push_back({neighborNode->mnId});
                    edgeConnection.push_back({node->mnId, neighborNode->mnId});
                }

                sNode.push(neighborNode->mnId);
                neighborNode = neighborNode->mpNext;
            }
        }

        vertexArray.emplace_back(std::move(vertexConnection));
        edgeArray.emplace_back(std::move(edgeConnection));
    }

    mbTraversed = true;
}

std::vector<std::vector<unsigned long>> Graph::GetVertexConnections() 
{
    std::vector<std::vector<unsigned long>> connections;
    std::unordered_set<unsigned long> visited;
    
    for(std::map<unsigned long, Node*>::const_iterator it=mmAdjList.begin(), iend = mmAdjList.end(); it!=iend; it++)
    {
        if(visited.count(it->second->mnId)) continue;
        std::stack<unsigned long> sNode;
        sNode.push(it->second->mnId);
        std::vector<unsigned long> connection;
        connection.reserve(mmAdjList.size());

        while(!sNode.empty())
        {
            unsigned long nextId = sNode.top();
            sNode.pop();

            if(visited.count(nextId))
                continue;
            visited.insert(nextId);
            connection.push_back(nextId);

            Node* node = mmAdjList[nextId];
            Node* neighborNode = node->mpNext;
            while(neighborNode!=nullptr)
            {
                if(visited.count(neighborNode->mnId)) 
                {
                    neighborNode = neighborNode->mpNext;
                    continue;
                }
                sNode.push(neighborNode->mnId);
                neighborNode = neighborNode->mpNext;
            }
        }
        
        connections.emplace_back(std::move(connection));
    }
    mbTraversed = true;

    return connections;
}

void Graph::GetMaxConnections(std::vector<Vertex>& vertexArray, std::vector<Edge>& edgeArray)
{
    int maxConnectivity = 0;
    
    std::unordered_set<unsigned long> visited;
    for(std::map<unsigned long, Node*>::const_iterator it=mmAdjList.begin(), iend = mmAdjList.end(); it!=iend; it++)
    {
        if(visited.count(it->second->mnId)) continue;
        std::stack<unsigned long> sNode;
        sNode.push(it->second->mnId);
        std::vector<Vertex> vertexConnection;
        std::vector<Edge> edgeConnection;
        vertexConnection.reserve(mnVertices);
        edgeConnection.reserve(mnEdges);

        while(!sNode.empty())
        {
            unsigned long nextId = sNode.top();
            sNode.pop();
            if(visited.count(nextId))
                continue;
            visited.insert(nextId);
            vertexConnection.push_back({nextId});

            Node* node = mmAdjList[nextId];
            Node* neighborNode = node->mpNext;
            while(neighborNode!=nullptr)
            {
                if(visited.count(neighborNode->mnId)) 
                {
                    neighborNode = neighborNode->mpNext;
                    continue;
                }

                if(node->mnId<neighborNode->mnId)
                    edgeConnection.push_back({node->mnId, neighborNode->mnId});
                
                sNode.push(neighborNode->mnId);
                neighborNode = neighborNode->mpNext;

            }
        }
        int connectivity = static_cast<int>(vertexConnection.size());
        if(connectivity>maxConnectivity)
        {
            maxConnectivity = connectivity;
            vertexArray = std::move(vertexConnection);
            edgeArray = std::move(edgeConnection);
        }
    }


    mbTraversed = true;
}

std::vector<unsigned long> Graph::GetMaxVertexConnection()
{
    std::vector<std::vector<unsigned long>> connections = GetVertexConnections();
    return *std::max_element(connections.begin(), connections.end(), [](auto& a, auto& b) -> bool {
        return a.size() < b.size();
    });
}

void Graph::SetAllNodesUnvisited()
{
    for(std::map<unsigned long, Node*>::const_iterator it=mmAdjList.begin(), iend = mmAdjList.end(); it!=iend; it++)
    {
        it->second->visited = false;
    }

    mbTraversed = false;
}

} // namespace ORB_SLAM3