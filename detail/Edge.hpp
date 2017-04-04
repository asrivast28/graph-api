/**
 * @file Edge.hpp
 * @brief Details of Edge and the corresponding iterator functions.
 */
#ifndef DETAIL_EDGE_HPP_
#define DETAIL_EDGE_HPP_

#include "Vertex.hpp"


/**
 * @brief  Constructor for the edge wrapper.
 *
 * @param graph  Instance of the graph implementation.
 * @param edge   Instance of the edge implementation.
 */
template <template <typename> class GraphType, typename VertexIdType>
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Edge(
  const GraphImpl& graph,
  const EdgeType& edge
) : m_graph(graph),
    m_edge(edge)
{
}

/**
 * @brief  Returns the source vertex of this edge.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename ::Vertex<GraphType, VertexIdType>
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::source(
) const
{
  return typename ::Vertex<GraphType, VertexIdType>(m_graph, boost::source(m_edge, m_graph));
}

/**
 * @brief  Returns the target vertex of this edge.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename ::Vertex<GraphType, VertexIdType>
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::target(
) const
{
  return typename ::Vertex<GraphType, VertexIdType>(m_graph, boost::target(m_edge, m_graph));
}

/**
 * @brief  Constructor for iterator over the edges.
 *
 * @param graph     Instance of the graph implementation.
 * @param vertices  Pair of begin and end iterator implementations over the edges.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename IteratorType>
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>::Iterator(
  const GraphImpl& graph,
  const std::pair<IteratorType, IteratorType>& edges
) : m_graph(graph),
    m_current(edges.first),
    m_end(edges.second)
{
}

/**
 * @brief  Increments the iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename IteratorType>
typename Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::template Iterator<IteratorType>&
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>::operator++(
)
{
  if (m_current != m_end) {
    ++m_current;
  }
  return *this;
}

/**
 * @brief  Checks if the iterator is same as another iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename IteratorType>
bool
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>::operator==(
  const Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename IteratorType>
bool
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>::operator!=(
  const Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the edge that the iterator is currently pointing to.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename IteratorType>
typename ::Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>
Edge<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator<IteratorType>::operator*(
)
{
  return Edge<GraphType, VertexIdType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the edge iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 * @param edges  Pair of begin and end iterator implementations over the edges.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
EdgeIterator<GraphType, VertexIdType, IteratorType, EnableBoost<GraphType, VertexIdType>>::EdgeIterator(
  const typename GraphType<VertexIdType>::Impl& graph,
  const std::pair<IteratorType, IteratorType>& edges
) : m_graph(graph),
    m_edges(edges)
{
}

/**
 * @brief  Returns the begin iterator over the edges.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
EdgeIterator<GraphType, VertexIdType, IteratorType, EnableBoost<GraphType, VertexIdType>>::begin(
) const
{
  return typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>(m_graph, m_edges);
}

/**
 * @brief  Returns the end iterator over the edges.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
EdgeIterator<GraphType, VertexIdType, IteratorType, EnableBoost<GraphType, VertexIdType>>::end(
) const
{
  IteratorType end = m_edges.second;
  return typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
}

#endif // DETAIL_EDGE_HPP_
