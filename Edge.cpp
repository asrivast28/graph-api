/**
 * @file Edge.cpp
 * @brief Implementation of Edge and the corresponding iterator functions.
 */
#include "Edge.hpp"

#include "GraphType.hpp"
#include "Vertex.hpp"


/**
 * @brief  Constructor for the edge wrapper.
 *
 * @param graph  Instance of the graph implementation.
 * @param edge   Instance of the edge implementation.
 */
template <typename GraphType>
Edge<GraphType>::Edge(
  const GraphImpl& graph,
  const EdgeType& edge
) : m_graph(graph),
    m_edge(edge)
{
}

/**
 * @brief  Returns the source vertex of this edge.
 */
template <>
typename ::Vertex<UndirectedGraphType<unsigned>>
Edge<UndirectedGraphType<unsigned>>::source(
) const
{
  return typename ::Vertex<UndirectedGraphType<unsigned>>(m_graph, boost::source(m_edge, m_graph));
}

/**
 * @brief  Returns the target vertex of this edge.
 */
template <>
typename ::Vertex<UndirectedGraphType<unsigned>>
Edge<UndirectedGraphType<unsigned>>::target(
) const
{
  return typename ::Vertex<UndirectedGraphType<unsigned>>(m_graph, boost::target(m_edge, m_graph));
}

/**
 * @brief  Constructor for iterator over the edges.
 *
 * @param graph     Instance of the graph implementation.
 * @param vertices  Pair of begin and end iterator implementations over the edges.
 */
template <typename GraphType>
template <typename IteratorType>
Edge<GraphType>::Iterator<IteratorType>::Iterator(
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
template <typename GraphType>
template <typename IteratorType>
typename Edge<GraphType>::template Iterator<IteratorType>&
Edge<GraphType>::Iterator<IteratorType>::operator++(
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
template <typename GraphType>
template <typename IteratorType>
bool
Edge<GraphType>::Iterator<IteratorType>::operator==(
  const Edge<GraphType>::Iterator<IteratorType>& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <typename GraphType>
template <typename IteratorType>
bool
Edge<GraphType>::Iterator<IteratorType>::operator!=(
  const Edge<GraphType>::Iterator<IteratorType>& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the edge that the iterator is currently pointing to.
 */
template <typename GraphType>
template <typename IteratorType>
typename ::Edge<GraphType>
Edge<GraphType>::Iterator<IteratorType>::operator*(
)
{
  return Edge<GraphType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the edge iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 * @param edges  Pair of begin and end iterator implementations over the edges.
 */
template <typename GraphType, typename IteratorType>
Edges<GraphType, IteratorType>::Edges(
  const GraphImpl& graph,
  const std::pair<IteratorType, IteratorType>& edges
) : m_graph(graph),
    m_edges(edges)
{
}

/**
 * @brief  Returns the begin iterator over the edges.
 */
template <typename GraphType, typename IteratorType>
typename Edge<GraphType>::template Iterator<IteratorType>
Edges<GraphType, IteratorType>::begin(
) const
{
  return typename Edge<GraphType>::template Iterator<IteratorType>(m_graph, m_edges);
}

/**
 * @brief  Returns the end iterator over the edges.
 */
template <typename GraphType, typename IteratorType>
typename Edge<GraphType>::template Iterator<IteratorType>
Edges<GraphType, IteratorType>::end(
) const
{
  IteratorType end = m_edges.second;
  return typename Edge<GraphType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
}

// Explicit instantiation.
template class Edge<UndirectedGraphType<unsigned>>;

template class Edge<UndirectedGraphType<unsigned>>::Iterator<UndirectedGraphType<unsigned>::EdgeIterator>;
template class Edges<UndirectedGraphType<unsigned>, UndirectedGraphType<unsigned>::EdgeIterator>;

template class Edge<UndirectedGraphType<unsigned>>::Iterator<UndirectedGraphType<unsigned>::InEdgeIterator>;
template class Edges<UndirectedGraphType<unsigned>, UndirectedGraphType<unsigned>::InEdgeIterator>;
