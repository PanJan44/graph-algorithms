#include "algorithms.h"
#include "fmt/include/fmt/base.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <stack>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

const int INF = std::numeric_limits<int>::max();

std::unordered_map<int, int> Algorithms::Dijkstra(const Graph &graph, int startVertex) {
  auto distances = graph.InitDistances(startVertex);

  auto compare = [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second > b.second;
  };

  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(compare)> pq(compare);
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
        if (distances[u] != INF) {
          distances[v] = std::min(distances[v], distances[u] + neighbour.weight);
        }
      }
    }
  }

  return distances;
}

std::vector<std::vector<int>> Algorithms::FloydWarshall(const Graph &graph) {
  auto distanceArray = graph.GetDistanceArray();

  for (auto i = 0; i < distanceArray.size(); i++) {
    for (auto y = 0; y < distanceArray.size(); y++) {
      for (auto x = 0; x < distanceArray.size(); x++) {
        if (distanceArray[y][i] != INF && distanceArray[x][i]) {
          distanceArray[x][y] = std::min(distanceArray[x][y], distanceArray[y][i] + distanceArray[x][i]);
        }
      }
    }
  }

  return distanceArray;
}

std::tuple<bool, std::unordered_set<int>, std::unordered_set<int>> Algorithms::IsBiparite(const Graph& graph, int startVertex) {
  auto distances = graph.InitDistances(startVertex);
  std::queue<int> q;
  q.push(startVertex);

  while(!q.empty()) {
    const auto u = q.front();
    q.pop();

    for(const auto& neighbour : graph.GetNeighbours(u)) {
      const auto v = neighbour.vertex;
      if(distances[v] == INF) {
        distances[v] = distances[u] + 1;
        q.push(v);
      }
      else if(distances[u] % 2 == distances[v] % 2) {
        return std::make_tuple(false, std::unordered_set<int>(), std::unordered_set<int>());
      }
    }
  }

  std::unordered_set<int> V1;
  std::unordered_set<int> V2;
  for(const auto& pair : distances) {
    if(pair.second % 2 == 0) {
      V1.insert(pair.first);
    }
    else {
      V2.insert(pair.first);
    }
  }

  return std::make_tuple(true, V1, V2);
}

std::vector<int> Algorithms::TopologicalSort(const Graph& graph) {
  std::unordered_map<int, int> inDegree;
  std::stack<int> zeroInDegreeStack;
  std::vector<int> result;

  for(const auto& pair : graph.GetAdjacencyList()) {
    inDegree[pair.first] = 0;
  }

  for(const auto& pair : graph.GetAdjacencyList()) {
    const auto u = pair.first;
    for(const auto& neighbour : graph.GetNeighbours(u)) {
      inDegree[neighbour.vertex]++;
    }
  }

  for(const auto& pair : inDegree) { 
    if(pair.second == 0) {
      zeroInDegreeStack.push(pair.first);
    }
  }

  while(!zeroInDegreeStack.empty()) {
    const auto u = zeroInDegreeStack.top();
    zeroInDegreeStack.pop();
    result.emplace_back(u);

    for(const auto& neighbour : graph.GetNeighbours(u)) {
      const auto v = neighbour.vertex;
      inDegree[v]--;
      if(inDegree[v] == 0) {
        zeroInDegreeStack.push(v);
      }
    }
  }

  return result;
}

void Algorithms::DFS(const Graph &graph, int startVertex, std::unordered_map<int, bool> &visited) {
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
