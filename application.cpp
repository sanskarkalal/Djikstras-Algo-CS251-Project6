/* -----------------------------------
Program 6: Graphs
Class: CS 251 Spring 2023
Author: Sanskar Kalal
----------------------------------- */


// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <set>
#include<algorithm>



#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

///@brief: This function takes in a string and returns a lowercase version of it
///@param: s string to be converted to lowercase
string lower(string s){
  string p = "";
  for(int i=0; i < s.size(); i++){
    p += tolower(s[i]);
  }
  return p;
}


///@brief: This function prints the data of the points and nodes
///@param: personOne, personTwo, destBuilding are the BuildingInfo structs of the people and destination
///@param: personOneNode, personTwoNode, destNode are the node IDs of the people and destination
void printData(BuildingInfo personOne, BuildingInfo personTwo, BuildingInfo destBuilding,
               long long personOneNode, long long personTwoNode, long long destNode, map<long long, Coordinates>& Nodes){

    cout << "Person 1's point:" << endl << " " << personOne.Fullname << endl;
    cout << " ( " << personOne.Coords.Lat << ", " << personOne.Coords.Lon << " )" << endl;
    cout << "Person 2's point:" << endl << " " << personTwo.Fullname << endl;
    cout << " ( " << personTwo.Coords.Lat << ", " << personTwo.Coords.Lon << " )" << endl;
    cout << "Destination Building:" << endl << " " << destBuilding.Fullname << endl;
    cout << " ( " << destBuilding.Coords.Lat << ", " << destBuilding.Coords.Lon << " )" << endl << endl;

    cout << "Nearest P1 node: " << endl << " " << personOneNode << endl;
    cout << " ( " << Nodes.at(personOneNode).Lat << ", " << Nodes.at(personOneNode).Lon << " )" << endl;
    cout << "Nearest P2 node: " << endl << " " << personTwoNode << endl;
    cout << " ( " << Nodes.at(personTwoNode).Lat << ", " << Nodes.at(personTwoNode).Lon << " )" << endl;
    cout << "Nearest Destination node: " << endl << " " << destNode << endl;
    cout << " ( " << Nodes.at(destNode).Lat << ", " << Nodes.at(destNode).Lon << " )" << endl;
}

///@brief: This function finds the building that the user inputs
///@param: personBuilding is the building that the user inputs
///@param: Buildings is the vector of BuildingInfo structs
bool findBuilding(string personBuilding, vector<BuildingInfo>& Buildings, BuildingInfo& personBuildingInfo) {

  for (long unsigned int i = 0; i < Buildings.size(); i++) {

    if(lower(Buildings[i].Abbrev) == lower(personBuilding)){
      personBuildingInfo = Buildings[i];
      return true;
    }else if(lower(Buildings[i].Fullname).find(lower(personBuilding)) != string::npos){
      personBuildingInfo = Buildings[i];
      return true;
    }
  }
  return false;
}

///@brief: This function finds the nearest node to the building that the user inputs
///@param: Nodes is the map of node IDs and their coordinates
///@param: Footways is the vector of FootwayInfo structs
long long findFootway(Coordinates coords, map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways){

  long long ID;
  double minimum = numeric_limits<double>::max();

  for(int i = 0; i < Footways.size(); i++){
    for(int j = 0; j < Footways.at(i).Nodes.size(); j++){
      long long tempID = Footways.at(i).Nodes.at(j);
      Coordinates tempCoords = Nodes.at(tempID);
    
      if(distBetween2Points(coords.Lat, coords.Lon, tempCoords.Lat, tempCoords.Lon) < minimum){
        ID = tempID;
        minimum = distBetween2Points(coords.Lat, coords.Lon, tempCoords.Lat, tempCoords.Lon);
      }
    }
  }
  return ID;
}


///@brief: This function finds the closest building to the center coordinates
///@param: center is the center coordinates
///@param: Buildings is the vector of BuildingInfo structs
///@param: dest is the BuildingInfo struct of the closest building
///@param: min is the minimum distance between the center and the closest building
void closestBuilding(Coordinates center, vector<BuildingInfo> & Buildings,BuildingInfo &dest, double &min){
    min = numeric_limits<double>::max();

    //loop through all buildings
    for(int i = 0; i < Buildings.size(); i++){
        double distance = distBetween2Points(center.Lat, center.Lon, Buildings.at(i).Coords.Lat, Buildings.at(i).Coords.Lon);

        //if distance is less than min, set min to distance and set dest to the building
        if(distance < min){
        min = distance;
        dest = Buildings.at(i);
        }
    }
}


///@brief: This function finds the closest node to the center coordinates
///@param: Nodes is the map of node IDs and their coordinates
///@param: Footways is the vector of FootwayInfo structs
///@param: G is the graph of the map
void BuildGraph(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, graph<long long, double>& G) {

    //loop through all nodes and add them to the graph
  for(const auto &i: Nodes){
      G.addVertex(i.first);
  }

  // loop through all footways and add edges between nodes
  for(const auto &i: Footways){

      //loop through all nodes in the footway and add edges between them
    for(int j = 0; j < i.Nodes.size()-1; j++){
      long long node1 = i.Nodes.at(j);
      long long node2 = i.Nodes.at(j+1);
      double distance = distBetween2Points(Nodes.at(node1).Lat, Nodes.at(node1).Lon, Nodes.at(node2).Lat, Nodes.at(node2).Lon);
      G.addEdge(node1, node2, distance);
      G.addEdge(node2, node1, distance);
    }
  }

}


///@brief: This function prints the path
///@param: path is the vector of nodes in the path
///@param: Nodes is the map of node IDs and their coordinates
void printPath(vector<long long>& path){
    cout << "Path: ";
    for(int i = 0; i < path.size()-1; i++){
       cout<< path.at(i)<< "->";
    }
    cout << path.at(path.size()-1) << endl;
}






///@brief: This function finds the shortest path between two nodes and fills the path vector with the nodes in the path
///@param G: graph of nodes
///@param start: starting node
///@param end: ending node
bool djikstrasPath(graph<long long,double> G, long long start, long long end, vector<long long>& path){

    // Initialize the distance and predecessor maps
    map<long long, double> distances;
    map<long long, long long> predecessors;
    map<long long, bool> visited;

    // Initialize the maps
    for(const auto &i: G.getVertices()){
        distances[i] = numeric_limits<double>::max();
        predecessors[i] = 0;
        visited[i] = false;
    }

    // Set the starting node's distance to 0
    distances[start] = 0;


    // While there are still unvisited nodes
    while(!visited.at(end)){

        long long currentV = 0;
        double min = numeric_limits<double>::max();

        // Find the unvisited node with the smallest distance
        for(const auto &i: G.getVertices()){
            if(!visited.at(i) && distances.at(i) < min){
                currentV = i;
                min = distances.at(i);
            }
        }
       // For each adjacent node
        for(const auto &i: G.neighbors(currentV)){


            double edgeWeight;
            G.getWeight(currentV, i,edgeWeight);

            //Calculate the distance from startV to adjV
            double alternativePathDistance = distances.at(currentV) + edgeWeight;

            //If the distance is less than the current distance, update the distance and predecessor
            if(alternativePathDistance < distances.at(i)){
                distances[i] = alternativePathDistance;
                predecessors[i] = currentV;
            }
        }
        //Mark the node as visited
        visited[currentV] = true;
    }

    // If the distance to the end node is infinity, there is no path
    if(distances.at(end) == numeric_limits<double>::max()){
        return false;
    }

    // Fill the path vector with the nodes in the path
    long long current = end;
    while(current != start){
        path.push_back(current);
        current = predecessors.at(current);
    }
    path.push_back(start);

    // Reverse the path vector so that it is in the correct order
    reverse(path.begin(), path.end());
    return true;
}



///@brief - Returns the length of the path
///@param path - The path to find the length of
///@param G - The graph to find the length of the path in
double pathLength(vector<long long>& path, graph<long long, double>& G){
    double length = 0.0;
    for(int i = 0; i < path.size()-1; i++){
        double edgeWeight;
        G.getWeight(path.at(i), path.at(i+1),edgeWeight);
        length += edgeWeight;
    }

    return length;
}



///@brief - This function finds the best place to meet between two people and prints the path to get there
///@param Nodes - The map of nodes
///@param Footways - The vector of footways
///@param Buildings - The vector of buildings
///@param G - The graph of nodes
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {

    string person1Building, person2Building;
    BuildingInfo person1BuildingInfo, person2BuildingInfo;

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);




  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Find the building info (including the full name) for person 1
    if(!findBuilding(person1Building, Buildings, person1BuildingInfo)){

    cout << "Person 1's building not found" << endl;

    }
    // Find the building info (including the full name) for person 2
    else if(!findBuilding(person2Building, Buildings, person2BuildingInfo)){

      cout << "Person 2's building not found" << endl;

    }else{

        bool destNotFound = false;
        vector <BuildingInfo> tempBuildings = Buildings;


        do {
            BuildingInfo destination;
            double minimumDistance;

            // If the buildings are the same, then the destination is that building
            if(person1BuildingInfo.Fullname == person2BuildingInfo.Fullname){
                destination = person1BuildingInfo;

            }else{
                Coordinates center = centerBetween2Points(person1BuildingInfo.Coords.Lat,person1BuildingInfo.Coords.Lon,person2BuildingInfo.Coords.Lat,person2BuildingInfo.Coords.Lon);
                closestBuilding(center, tempBuildings, destination, minimumDistance);
            }

            // Find the sources and destination nodes
            long long source1 = findFootway(person1BuildingInfo.Coords,Nodes,Footways);
            long long dest = findFootway(destination.Coords,Nodes,Footways);
            long long source2 = findFootway(person2BuildingInfo.Coords,Nodes,Footways);
            printData(person1BuildingInfo, person2BuildingInfo, destination,source1,source2,dest,Nodes);




            vector<long long> path1,path2;

            // Find the paths from the sources to the destination
            if(!djikstrasPath(G,source1,dest,path1) || !djikstrasPath(G,source2,dest,path2)){

                // If a path is not found, remove the building from the list of buildings and try again
                tempBuildings.pop_back();
                cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
                destNotFound = true;
            }else{

                // If a path is found, print the paths and the distances
                if(destNotFound){
                    cout << "New destination building: " << destination.Fullname << endl;
                    cout << " ( " << destination.Coords.Lat << ", " << destination.Coords.Lon << " )" << endl;
                    cout << "Nearest destination node" << dest << endl;
                    cout << " ( " << Nodes.at(dest).Lat << ", " << Nodes.at(dest).Lon << " )" << endl;
                    cout << endl;
                    cout << "Person 1's distance to dest: "<< pathLength(path1, G) << " miles"<<endl;
                    printPath(path1);
                    cout << "Person 2's distance to dest: " << pathLength(path2, G)<< " miles"<<endl;
                    printPath(path2);
                }else{
                    cout << endl;
                    cout << "Person 1's distance to dest: "<< pathLength(path1, G) << " miles"<<endl;
                    printPath(path1);
                    cout << "Person 2's distance to dest: " << pathLength(path2, G)<< " miles"<<endl;
                    printPath(path2);
                }

            }
        }while(destNotFound && tempBuildings.size() > 0); // Loop until a path is found

        // If a path is not found, print an error message
        if(destNotFound){
            cout << "No destination building found" << endl;
        }

    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


    BuildGraph(Nodes, Footways, G);

   cout << "# of vertices: " << G.NumVertices() << endl;
   cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
