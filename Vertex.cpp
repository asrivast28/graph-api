/**
 * @file Vertex.cpp
 * @brief Implementation of Vertex and the corresponding iterator functions.
 */
#include "Vertex.hpp"


/**
 * @brief  Constructor for the vertex wrapper.
 *
 * @param graph   Instance of the graph implementation.
 * @param vertex  Instance of the vertex implementation.
 */
template <template <typename> class GraphType, typename VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Vertex(
  const GraphImpl& graph,
  const VertexType& vertex
) : m_graph(graph),
    m_vertex(vertex)
{
}

/**
 * @brief  Returns the id of this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::id(
) const
{
  return m_graph[m_vertex].id;
}

/**
 * @brief  Returns the in degree of this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
size_t
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::inDegree(
) const
{
  return static_cast<size_t>(boost::in_degree(m_vertex, m_graph));
}

/**
 * @brief  Returns an iterator over the edges incident on this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename ::EdgeIterator<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::inEdges(
) const
{
  return typename ::EdgeIterator<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, m_graph));
}

/**
 * @brief  Checks if this vertex has an edge to the given vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::hasEdgeTo(
  const Vertex<GraphType, VertexIdType>& other
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
template <template <typename> class GraphType, typename VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator::Iterator(
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
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator&
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator::operator++(
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
bool
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator::operator==(
  const Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator::operator!=(
  const Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the vertex that the iterator is currently pointing to.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename ::Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>
Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Iterator::operator*(
)
{
  return Vertex<GraphType, VertexIdType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the vertex iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIterator<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::VertexIterator(
  const typename GraphType<VertexIdType>::Impl& graph
) : m_graph(graph)
{
}

/**
 * @brief  Returns the begin iterator over the vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType>::Iterator
VertexIterator<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::begin(
) const
{
  return typename Vertex<GraphType, VertexIdType>::Iterator(m_graph, boost::vertices(m_graph));
}

/**
 * @brief  Returns the end iterator over the vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType>::Iterator
VertexIterator<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::end(
) const
{
  IteratorType end = boost::vertices(m_graph).second;
  return typename Vertex<GraphType, VertexIdType>::Iterator(m_graph, std::make_pair(end, end));
}

// Explicit instantiation.
template class Vertex<UndirectedAdjacencyList, unsigned>;
template class Vertex<BidirectionalAdjacencyList, unsigned>;

template class Vertex<UndirectedAdjacencyList, size_t>;
template class Vertex<BidirectionalAdjacencyList, size_t>;

template class VertexIterator<UndirectedAdjacencyList, unsigned>;
template class VertexIterator<BidirectionalAdjacencyList, unsigned>;

template class VertexIterator<UndirectedAdjacencyList, size_t>;
template class VertexIterator<BidirectionalAdjacencyList, size_t>;
