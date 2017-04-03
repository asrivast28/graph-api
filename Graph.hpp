/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "GraphFile.hpp"
#include "GraphType.hpp"
#include "Vertex.hpp"

#include <unordered_map>
#include <vector>


/**
 * @brief  Class that provides graph functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename Enable = void>
class Graph;

/**
 * @brief  Partial specialization of Graph class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>> {
public:
  using Vertex = typename ::Vertex<GraphType, VertexIdType>;
  using Edge = typename ::Edge<GraphType, VertexIdType>;

public:
  Graph(const std::string&, const enum GraphFileType);

  VertexIterator<GraphType, VertexIdType>
  vertices() const;

  VertexIdType
  vertexCount() const;

  Vertex
  getVertexFromId(const VertexIdType&) const;

  VertexIdType
  maxVertexId() const;

  EdgeIterator<GraphType, VertexIdType>
  edges() const;

  size_t
  edgeCount() const;

  bool
  edgeExists(const Vertex&, const Vertex&) const;

  ~Graph();

private:
  void
  build(const std::vector<std::pair<VertexIdType, VertexIdType>>&, const std::unordered_set<VertexIdType>&);

private:
  typename GraphType<VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename GraphType<VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>

#endif // GRAPH_HPP_
