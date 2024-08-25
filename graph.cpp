#include "graph.h"
#include "./fmt/include/fmt/core.h"
#include <limits>

void Graph::AddConnection(int u, int v, int weight, bool directed) {
  adjacencyList[u].emplace_back(Node(v, weight));

  if(directed) {
    adjacencyList[v].emplace_back(Node(u, weight));
  }
}

void Graph::AddConnectionArray(int u, int v, int weight, bool directed) {
  distanceArray[u][v] = weight;

  if(directed) {
    distanceArray[v][u] = weight;
  }
}

void Graph::PrintGraph() const {
  for (const auto &pair : adjacencyList) {
    for (const auto &neighbour : pair.second) {
      fmt::print("Vertex: {0} -> {1} weight of {2}\n", pair.first, neighbour.vertex, neighbour.weight);
    }
  }
}

std::unordered_map<int, int> Graph::InitDistances(int startVertex) const {
  auto distances = std::unordered_map<int, int>();
  for (const auto &pair : adjacencyList) {
    distances[pair.first] = std::numeric_limits<int>::max();

    for (const auto &neighbor : pair.second) {
      int neighbour = neighbor.vertex;
      if (distances.find(neighbour) == distances.end()) {
        distances[neighbour] = std::numeric_limits<int>::max();
      }
    }
  }

  distances[startVertex] = 0;
  return distances;
}

std::vector<Graph::Node> Graph::GetNeighbours(int vertex) const {
  const auto it = adjacencyList.find(vertex);
  if(it != adjacencyList.end())
    return it->second;

  return std::vector<Graph::Node>();
}
