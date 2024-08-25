#ifndef GRAPH_H
#define GRAPH_H

#include <limits>
#include <unordered_map>
#include <vector>

class Graph {
private:
  struct Node {
    int vertex;
    int weight;

  public:
    Node(int vertex, int weight) : vertex(vertex), weight(weight) {}
  };

  std::unordered_map<int, std::vector<Node>> adjacencyList;
  std::vector<std::vector<int>> distanceArray;

public:
  Graph() = default;
  Graph(int verticesCount) { distanceArray.resize(verticesCount, std::vector<int>(verticesCount, std::numeric_limits<int>::max()));}
  void PrintGraph() const;
  void AddConnection(int u, int v, int weight, bool directed = false);
  void AddConnectionArray(int u, int v, int weight, bool directed);
  std::unordered_map<int, int> InitDistances(int startVertex) const;
  std::vector<Node> GetNeighbours(int vertex) const;
  std::unordered_map<int, std::vector<Node>> GetAdjacencyList() const { return adjacencyList; }
  std::vector<std::vector<int>> GetDistanceArray() const { return distanceArray; }
};

#endif
