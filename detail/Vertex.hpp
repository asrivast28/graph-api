/**
 * @file Vertex.hpp
 * @brief Details of Vertex and the corresponding iterator functions.
 */
#ifndef DETAIL_VERTEX_HPP_
#define DETAIL_VERTEX_HPP_


/**
 * @brief  Default constructor for the vertex wrapper.
 */
template <template <typename> class GraphType, typename VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Vertex(
) : m_graph(nullptr),
    m_vertex(0)
{
}

/**
 * @brief  Constructor for the vertex wrapper.
 *
 * @param graph   Instance of the graph implementation.
 * @param vertex  Instance of the vertex implementation.
 */
template <template <typename> class GraphType, typename VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Vertex(
  const GraphImpl* const graph,
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
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::id(
) const
{
  return (*m_graph)[m_vertex].id;
}

/**
 * @brief  Returns the in degree of this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::inDegree(
) const
{
  return boost::in_degree(m_vertex, *m_graph);
}

/**
 * @brief  Returns an iterator provider over the edges incident on this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::inEdges(
) const
{
  return EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, *m_graph));
}

/**
 * @brief  Returns the out degree of this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::outDegree(
) const
{
  return boost::out_degree(m_vertex, *m_graph);
}

/**
 * @brief  Returns an iterator provider over the outgoing edges from this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexIdType>::OutEdgeIterator>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::outEdges(
) const
{
  return EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexIdType>::OutEdgeIterator>(m_graph, boost::out_edges(m_vertex, *m_graph));
}

/**
 * @brief  Returns a provider for all the simple paths of a given length starting from this vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
SimplePathProvider<GraphType, VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::simplePaths(
  const VertexIdType& l
) const
{
  return SimplePathProvider<GraphType, VertexIdType>(*this, l);
}

/**
 * @brief  Checks if this vertex has an edge to the given vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::hasEdgeTo(
  const Vertex<GraphType, VertexIdType>& other
) const
{
  return boost::edge(m_vertex, other.m_vertex, *m_graph).second;
}

/**
 * @brief  Constructor for iterator over the vertices.
 *
 * @param graph     Instance of the graph implementation.
 * @param vertices  Pair of begin and end iterator implementations over the vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::Iterator(
  const GraphImpl* const graph,
  const std::pair<IteratorType, IteratorType>& vertices
) : m_graph(graph),
    m_current(vertices.first),
    m_end(vertices.second)
{
}

/**
 * @brief  Increments the iterator using the prefix increment iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator&
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::operator++(
)
{
  if (m_current != m_end) {
    ++m_current;
  }
  return *this;
}

/**
 * @brief  Increments the iterator using the postfix increment iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::operator++(
  int
)
{
  auto copy = *this;
  operator++();
  return copy;
}

/**
 * @brief  Checks if the iterator is same as another iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::operator==(
  const Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::operator!=(
  const Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the vertex that the iterator is currently pointing to.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename ::Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>
Vertex<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Iterator::operator*(
) const
{
  return Vertex<GraphType, VertexIdType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the vertex iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIteratorProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::VertexIteratorProvider(
  const typename GraphType<VertexIdType>::Impl* const graph
) : m_graph(graph)
{
}

/**
 * @brief  Returns the begin iterator over the vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType>::Iterator
VertexIteratorProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::begin(
) const
{
  return typename Vertex<GraphType, VertexIdType>::Iterator(m_graph, boost::vertices(*m_graph));
}

/**
 * @brief  Returns the end iterator over the vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
typename Vertex<GraphType, VertexIdType>::Iterator
VertexIteratorProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::end(
) const
{
  IteratorType end = boost::vertices(*m_graph).second;
  return typename Vertex<GraphType, VertexIdType>::Iterator(m_graph, std::make_pair(end, end));
}

#endif // DETAIL_VERTEX_HPP_
