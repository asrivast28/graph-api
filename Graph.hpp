/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "GraphFile.hpp"
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
template <template <typename> class GraphType, typename UnsignedType>
class Graph<GraphType, UnsignedType, EnableBoostAll<GraphType, UnsignedType>> {
public:
  using VertexIdType = UnsignedType;

  using Vertex = typename ::Vertex<GraphType, VertexIdType>;
  using Edge = typename ::Edge<GraphType, VertexIdType>;

public:
  Graph(const std::string&, const enum GraphFileType);

  bool
  isDirected() const;

  VertexIterator<GraphType, VertexIdType>
  vertices() const;

  template <typename Comparator>
  std::vector<Vertex>
  sorted(Comparator&&) const;

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
  typename GraphType<VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename GraphType<VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<GraphType, UnsignedType, EnableBoostAll<GraphType, UnsignedType>>


#include "detail/GraphFile.hpp"
#include "detail/Edge.hpp"
#include "detail/Vertex.hpp"
#include "detail/Graph.hpp"

#endif // GRAPH_HPP_
