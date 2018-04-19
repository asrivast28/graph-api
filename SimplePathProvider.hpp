/**
 * @file SimplePathProvider.hpp
 * @brief Declaration of simple path provider funcions.
 */
#ifndef SIMPLEPATHPROVIDER_HPP_
#define SIMPLEPATHPROVIDER_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "Vertex.hpp"


/**
 * @brief  Class that provides simple path provider functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename Enable = void>
class SimplePathProvider;

/**
 * @brief  Partial specialization of SimplePathProvider class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType>
class SimplePathProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>> {
private:
  using Vertex = typename ::Vertex<GraphType, VertexIdType>;
  using OutEdgeIterator = typename ::Edge<GraphType, VertexIdType>::OutEdgeIterator;

public:
  SimplePathProvider(const Vertex&, const VertexIdType&);

  std::vector<VertexIdType>
  next();

private:
  std::vector<std::pair<OutEdgeIterator, OutEdgeIterator>> m_stack;
  std::vector<VertexIdType> m_visited;
  const VertexIdType m_pathLength;
}; // class SimplePathProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, UnsignedType>>

#endif // SIMPLEPATHPROVIDER_HPP_
