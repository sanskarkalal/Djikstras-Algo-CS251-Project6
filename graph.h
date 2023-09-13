/* -----------------------------------
Program 6: Graphs
Class: CS 251 Spring 2023
Author: Sanskar Kalal
----------------------------------- */

// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_set>
#include <map>
using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:

    map<VertexT, map<VertexT,WeightT>> adjList;
    vector<VertexT> vertices;

 public:

  graph() {}

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return vertices.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {

    int count = 0;
    for (unsigned int i = 0; i < vertices.size(); ++i) {
        count += adjList.at(vertices[i]).size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    if (adjList.find(v) == adjList.end()) {
        adjList[v];
        vertices.push_back(v);
        return true;
    }
    return false;


  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {

      // if vertex does not exists in the graph
    if (adjList.find(from) == adjList.end() || adjList.find(to) == adjList.end()) {
        return false;
    }else{

        // add edge to the graph
        adjList[from][to] = weight;
        return true;
    }

  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {

      // if vertex does exists in the graph
    if(adjList.count(from) != 0 && adjList.at(from).count(to) != 0) {
        weight = adjList.at(from).at(to);
        return true;
    }
    return false;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> neighbors;

    // if vertex does exists in the graph
    if(adjList.count(v) != 0) {

        // add all the neighbors to the set
        for (auto it = adjList.at(v).begin(); it != adjList.at(v).end(); ++it) {
            neighbors.insert(it->first);
        }
    }
    return neighbors;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
      return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream &output) const
  {

      output << "***************************************************" << endl;
      output << "********************* GRAPH ***********************" << endl;

      output << "**Num vertices: " << adjList.size() << endl;
      int count = 0;
      for (auto &a : adjList)
      {
          count = count + a.second.size();
      }

      output << "**Num edges: " << count << endl;

      output << endl;
      output << "**Vertices:" << endl;
      for (int i = 0; i < adjList.size(); ++i)
      {
          output << " " << i + 1 << ". " << this->Vertices[i] << endl;
      }

      output << endl;
      output << "**Edges:" << endl;

      for (auto &a : adjList)
      {
          output << a.first << ":";
          for (auto &b : a.second)
          {
              output << "(" << a.first << ", " << b.first << ", " << b.second << ") ";
          }
          output << endl;
      }

      output << "**************************************************" << endl;
  }
};
