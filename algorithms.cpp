#include "algorithms.h"
#include "fmt/include/fmt/base.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

std::unordered_map<int, int> Algorithms::Dijkstra(const Graph &graph, int startVertex) {
  auto distances = graph.InitDistances(startVertex);

  // TODO implementation in graph class
  auto compare = [](const std::pair<int, int> &a,
                    const std::pair<int, int> &b) {
    return a.second > b.second;
  };

  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      decltype(compare)>
      pq(compare);
  pq.push({startVertex, 0});

  while (!pq.empty()) {
    auto examinedVertex = pq.top().first;
    auto examinedVertexDistance = pq.top().second;
    pq.pop();

    fmt::print("examinedVertex {0}, distance: {1}\n", examinedVertex, examinedVertexDistance);

    if (examinedVertexDistance > distances[examinedVertex]) {
      continue;
    }

    for (const auto &neighbour : graph.GetNeighbours(examinedVertex)) {
      const auto newDistance = distances[examinedVertex] + neighbour.weight;
      if (newDistance < distances[neighbour.vertex]) {
        fmt::println("I am pushing: neighbour {0}, distance: {1}", neighbour.vertex, newDistance);
        distances[neighbour.vertex] = newDistance;
        pq.push({neighbour.vertex, newDistance});
      }
    }
  }

  return distances;
}

std::unordered_map<int, int> Algorithms::BellmanFord(const Graph &graph, int startVertex) {
  auto distances = graph.InitDistances(startVertex);

  for (auto i = 0; i < distances.size() - 1; ++i) {
    for (const auto &pair : graph.GetAdjacencyList()) {
      const auto u = pair.first;
      for (const auto &neighbour : pair.second) {
        const auto v = neighbour.vertex;
        if (distances[u] != std::numeric_limits<int>::max()) {
          distances[v] = std::min(distances[v], distances[u] + neighbour.weight);
        }
      }
    }

    fmt::println("distances after {0} iteration: ", i + 1);
    for(const auto &pair : distances)
      fmt::println("distances[{0}] = {1}: ", pair.first, pair.second);
  }

  return distances;
}

void Algorithms::DFS(const Graph &graph, int startVertex,
                     std::unordered_map<int, bool> &visited) {
  visited[startVertex] = true;
  fmt::println("visiting vertex: {0}", startVertex);

  for (const auto &neighbour : graph.GetNeighbours(startVertex)) {
    if (!visited[neighbour.vertex]) {
      DFS(graph, neighbour.vertex, visited);
    }
  }
}

void Algorithms::BFS(const Graph &graph, int startVertex) {
  std::unordered_map<int, bool> visited;
  std::queue<int> queue;
  visited[startVertex] = true;
  queue.push(startVertex);

  while (!queue.empty()) {
    auto vertex = queue.front();
    queue.pop();
    fmt::println("visiting vertex: {0}", vertex);

    for (const auto &neighbour : graph.GetNeighbours(vertex)) {
      if (!visited[neighbour.vertex]) {
        visited[neighbour.vertex] = true;
        queue.push(neighbour.vertex);
      }
    }
  }
}
