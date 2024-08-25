#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Algorithms {
public:
  static std::vector<int> TopologicalSort(const Graph& graph);
  static std::vector<std::vector<int>> FloydWarshall(const Graph& graph);
  static std::unordered_map<int, int> Dijkstra(const Graph& graph, int startVertex);
  static std::unordered_map<int, int> BellmanFord(const Graph& graph, int startVertex);
  static std::tuple<bool, std::unordered_set<int>, std::unordered_set<int>> IsBiparite(const Graph& graph, int startVertex);
  static void DFS(const Graph& graph, int startVertex, std::unordered_map<int, bool>& visited);
  static void BFS(const Graph& graph, int startVertex);

private:
  std::unordered_map<int, int> InitDistances() const;
};

#endif
