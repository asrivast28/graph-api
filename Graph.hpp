/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "Vertex.hpp"

#include <unordered_map>
#include <vector>


/**
 * @brief  Class that provides general graph functionality.
 *
 * @tparam GraphType  Type of the graph implementation.
 */
template <typename GraphType>
class Graph {
public:
  using Vertex = typename ::Vertex<GraphType>;
  using Edge = typename ::Edge<GraphType>;

  using VertexIdType = typename GraphType::VertexIdType;

public:
  Graph(const std::string&);

  Vertices<GraphType>
  vertices() const;

  size_t
  vertexCount() const;

  Vertex
  getVertexFromId(const VertexIdType&) const;

  VertexIdType
  maxVertexId() const;

  Edges<GraphType>
  edges() const;

  size_t
  edgeCount() const;

  bool
  edgeExists(const Vertex&, const Vertex&) const;

  ~Graph();

private:
  typename GraphType::Impl m_graph;
  std::unordered_map<VertexIdType, typename GraphType::VertexType> m_idVertexMap;
}; // class Graph

#endif // GRAPH_HPP_
