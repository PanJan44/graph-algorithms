#ifndef GRAPH_H
#define GRAPH_H

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

public:
  Graph() = default;
  void AddConnection(int u, int v, int weight);
  void PrintGraph() const;
  std::unordered_map<int, int> InitDistances(int startVertex) const;
  std::vector<Node> GetNeighbours(int vertex) const;
  std::unordered_map<int, std::vector<Node>> GetAdjacencyList() const { return adjacencyList; }
};

#endif
