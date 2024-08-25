#include "algorithms.h"
#include "fmt/include/fmt/base.h"
#include "graph.h"
#include "fmt/include/fmt/ranges.h"

std::string MapVertex(int vertex) {
  switch(vertex){
    case 1:
      return "A";
    break;
    case 2:
      return "B";
    break;
    case 3:
      return "C";
    break;
    case 4:
      return "D";
    break;
    case 5:
      return "E";
    break;
    default: 
      return "";
  }
}

int main(int argc, char *argv[]) {
  const auto startVertex = 1;
  Graph graph;

  graph.AddConnection(1, 2, 6); //A -> B
  graph.AddConnection(1, 4, 1); //A -> D
  graph.AddConnection(2, 3, 5); //B -> C
  graph.AddConnection(4, 5, 1); //D -> E
  graph.AddConnection(4, 2, 2); //D -> B
  graph.AddConnection(5, 2, 2); //E -> B
  graph.AddConnection(5, 3, 5); //E -> C

  Graph graph2;
  graph2.AddConnection(1, 2, 4);
  graph2.AddConnection(1, 3, 3);
  graph2.AddConnection(2, 4, -5);
  graph2.AddConnection(2, 3, -2);
  graph2.AddConnection(3, 2, 1);
  graph2.AddConnection(4, 3, 2);

  Graph graph3;
  graph3.AddConnection(1, 2, 1, true);
  graph3.AddConnection(1, 3, 1, true);
  graph3.AddConnection(1, 4, 1, true);
  graph3.AddConnection(1, 6, 1, true);
  graph3.AddConnection(2, 5, 1, true);
  graph3.AddConnection(3, 5, 1, true);
  graph3.AddConnection(3, 7, 1, true);
  graph3.AddConnection(4, 7, 1, true);
  graph3.AddConnection(5, 6, 1, true);
  graph3.AddConnection(6, 7, 1, true);

  const auto [isBiparite, V1, V2] = Algorithms::IsBiparite(graph3, startVertex);
  fmt::println("Is biparite: {0}", isBiparite);

  fmt::print("V1: {}\n", fmt::join(V1, ", "));
  fmt::print("V2: {}\n", fmt::join(V2, ", "));

  /*const auto res = Algorithms::BellmanFord(graph2, startVertex);*/
  /*std::cout<<std::endl;*/
  /*for(const auto& pair : res) {*/
  /*  fmt::print("Vertex: {0} -> {1} distance: {2}\n", startVertex, pair.first, pair.second);*/
  /*}*/

  return 0;
}
