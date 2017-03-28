/**
 * @file Edge.cpp
 * @brief Implementation of Edge and the corresponding iterator functions.
 */
#include "Edge.hpp"

#include "Vertex.hpp"


/**
 * @brief  Constructor for the edge wrapper.
 *
 * @param graph  Instance of the graph implementation.
 * @param edge   Instance of the edge implementation.
 */
template <typename VertexIdType>
Edge<UndirectedAdjacencyList, VertexIdType>::Edge(
  const GraphImpl& graph,
  const EdgeType& edge
) : m_graph(graph),
    m_edge(edge)
{
}

/**
 * @brief  Returns the source vertex of this edge.
 */
template <typename VertexIdType>
typename ::Vertex<UndirectedAdjacencyList, VertexIdType>
Edge<UndirectedAdjacencyList, VertexIdType>::source(
) const
{
  return typename ::Vertex<UndirectedAdjacencyList, VertexIdType>(m_graph, boost::source(m_edge, m_graph));
}

/**
 * @brief  Returns the target vertex of this edge.
 */
template <typename VertexIdType>
typename ::Vertex<UndirectedAdjacencyList, VertexIdType>
Edge<UndirectedAdjacencyList, VertexIdType>::target(
) const
{
  return typename ::Vertex<UndirectedAdjacencyList, VertexIdType>(m_graph, boost::target(m_edge, m_graph));
}

/**
 * @brief  Constructor for iterator over the edges.
 *
 * @param graph     Instance of the graph implementation.
 * @param vertices  Pair of begin and end iterator implementations over the edges.
 */
template <typename VertexIdType>
template <typename IteratorType>
Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>::Iterator(
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
template <typename VertexIdType>
template <typename IteratorType>
typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>&
Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>::operator++(
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
template <typename VertexIdType>
template <typename IteratorType>
bool
Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>::operator==(
  const Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <typename VertexIdType>
template <typename IteratorType>
bool
Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>::operator!=(
  const Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the edge that the iterator is currently pointing to.
 */
template <typename VertexIdType>
template <typename IteratorType>
typename ::Edge<UndirectedAdjacencyList, VertexIdType>
Edge<UndirectedAdjacencyList, VertexIdType>::Iterator<IteratorType>::operator*(
)
{
  return Edge<UndirectedAdjacencyList, VertexIdType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the edge iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 * @param edges  Pair of begin and end iterator implementations over the edges.
 */
template <typename VertexIdType, typename IteratorType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType, IteratorType>::EdgeIterator(
  const typename UndirectedAdjacencyList<VertexIdType>::Impl& graph,
  const std::pair<IteratorType, IteratorType>& edges
) : m_graph(graph),
    m_edges(edges)
{
}

/**
 * @brief  Returns the begin iterator over the edges.
 */
template <typename VertexIdType, typename IteratorType>
typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType, IteratorType>::begin(
) const
{
  return typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>(m_graph, m_edges);
}

/**
 * @brief  Returns the end iterator over the edges.
 */
template <typename VertexIdType, typename IteratorType>
typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType, IteratorType>::end(
) const
{
  IteratorType end = m_edges.second;
  return typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
}

// Explicit instantiation.
template class Edge<UndirectedAdjacencyList, unsigned>;
template class Edge<UndirectedAdjacencyList, size_t>;

template class Edge<UndirectedAdjacencyList, unsigned>::Iterator<UndirectedAdjacencyList<unsigned>::EdgeIterator>;
template class Edge<UndirectedAdjacencyList, size_t>::Iterator<UndirectedAdjacencyList<size_t>::EdgeIterator>;

template class Edge<UndirectedAdjacencyList, unsigned>::Iterator<UndirectedAdjacencyList<unsigned>::InEdgeIterator>;
template class Edge<UndirectedAdjacencyList, size_t>::Iterator<UndirectedAdjacencyList<size_t>::InEdgeIterator>;

template class EdgeIterator<UndirectedAdjacencyList, unsigned, UndirectedAdjacencyList<unsigned>::EdgeIterator>;
template class EdgeIterator<UndirectedAdjacencyList, size_t, UndirectedAdjacencyList<size_t>::EdgeIterator>;

template class EdgeIterator<UndirectedAdjacencyList, unsigned, UndirectedAdjacencyList<unsigned>::InEdgeIterator>;
template class EdgeIterator<UndirectedAdjacencyList, size_t, UndirectedAdjacencyList<size_t>::InEdgeIterator>;
