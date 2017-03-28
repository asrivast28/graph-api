/**
 * @file Vertex.cpp
 * @brief Implementation of Vertex and the corresponding iterator functions.
 */
#include "Vertex.hpp"

#include "Edge.hpp"

template <template <typename> class GraphType, typename VertexIdType>
bool
VertexSkeleton<GraphType, VertexIdType>::hasEdgeTo(
  const VertexSkeleton<GraphType, VertexIdType>&
) const
{
  return false;
}

/**
 * @brief  Constructor for the vertex wrapper.
 *
 * @param graph   Instance of the graph implementation.
 * @param vertex  Instance of the vertex implementation.
 */
template <typename VertexIdType>
Vertex<UndirectedAdjacencyList, VertexIdType>::Vertex(
  const GraphImpl& graph,
  const VertexType& vertex
) : m_graph(graph),
    m_vertex(vertex)
{
}

/**
 * @brief  Returns the id of this vertex.
 */
template <typename VertexIdType>
VertexIdType
Vertex<UndirectedAdjacencyList, VertexIdType>::id(
) const
{
  return m_graph[m_vertex].id;
}

/**
 * @brief  Returns the in degree of this vertex.
 */
template <typename VertexIdType>
size_t
Vertex<UndirectedAdjacencyList, VertexIdType>::inDegree(
) const
{
  return static_cast<size_t>(boost::in_degree(m_vertex, m_graph));
}

/**
 * @brief  Returns an iterator over the edges incident on this vertex.
 */
template <typename VertexIdType>
typename ::EdgeIterator<UndirectedAdjacencyList, VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::InEdgeIterator>
Vertex<UndirectedAdjacencyList, VertexIdType>::inEdges(
) const
{
  return typename ::EdgeIterator<UndirectedAdjacencyList, VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, m_graph));
}

/**
 * @brief  Checks if this vertex has an edge to the given vertex.
 */
template <typename VertexIdType>
bool
Vertex<UndirectedAdjacencyList, VertexIdType>::hasEdgeTo(
  const Vertex<UndirectedAdjacencyList, VertexIdType>& other
) const
{
  return boost::edge(m_vertex, other.m_vertex, m_graph).second;
}

/**
 * @brief  Constructor for iterator over the vertices.
 *
 * @param graph     Instance of the graph implementation.
 * @param vertices  Pair of begin and end iterator implementations over the vertices.
 */
template <typename VertexIdType>
Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator::Iterator(
  const GraphImpl& graph,
  const std::pair<IteratorType, IteratorType>& vertices
) : m_graph(graph),
    m_current(vertices.first),
    m_end(vertices.second)
{
}

/**
 * @brief  Increments the iterator.
 */
template <typename VertexIdType>
typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator&
Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator::operator++(
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
bool
Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator::operator==(
  const Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <typename VertexIdType>
bool
Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator::operator!=(
  const Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the vertex that the iterator is currently pointing to.
 */
template <typename VertexIdType>
typename ::Vertex<UndirectedAdjacencyList, VertexIdType>
Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator::operator*(
)
{
  return Vertex<UndirectedAdjacencyList, VertexIdType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the vertex iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 */
template <typename VertexIdType>
VertexIterator<UndirectedAdjacencyList, VertexIdType>::VertexIterator(
  const typename UndirectedAdjacencyList<VertexIdType>::Impl& graph
) : m_graph(graph)
{
}

/**
 * @brief  Returns the begin iterator over the vertices.
 */
template <typename VertexIdType>
typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator
VertexIterator<UndirectedAdjacencyList, VertexIdType>::begin(
) const
{
  return typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator(m_graph, boost::vertices(m_graph));
}

/**
 * @brief  Returns the end iterator over the vertices.
 */
template <typename VertexIdType>
typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator
VertexIterator<UndirectedAdjacencyList, VertexIdType>::end(
) const
{
  IteratorType end = boost::vertices(m_graph).second;
  return typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator(m_graph, std::make_pair(end, end));
}

// Explicit instantiation.
template class Vertex<UndirectedAdjacencyList, unsigned>;
template class Vertex<UndirectedAdjacencyList, size_t>;
template class VertexIterator<UndirectedAdjacencyList, unsigned>;
template class VertexIterator<UndirectedAdjacencyList, size_t>;
