/**
 * @file Vertex.cpp
 * @brief Implementation of Vertex and the corresponding iterator functions.
 */
#include "Vertex.hpp"

#include "Edge.hpp"
#include "GraphType.hpp"


/**
 * @brief  Constructor for the vertex wrapper.
 *
 * @param graph   Instance of the graph implementation.
 * @param vertex  Instance of the vertex implementation.
 */
template <typename GraphType>
Vertex<GraphType>::Vertex(
  const GraphImpl& graph,
  const VertexType& vertex
) : m_graph(graph),
    m_vertex(vertex)
{
}

/**
 * @brief  Returns the id of this vertex.
 */
template <>
typename UndirectedGraphType<unsigned>::VertexIdType
Vertex<UndirectedGraphType<unsigned>>::id(
) const
{
  return m_graph[m_vertex].id;
}

/**
 * @brief  Returns the in degree of this vertex.
 */
template <>
size_t
Vertex<UndirectedGraphType<unsigned>>::inDegree(
) const
{
  return boost::in_degree(m_vertex, m_graph);
}

/**
 * @brief  Returns an iterator over the edges incident on this vertex.
 */
template <>
typename ::Edges<UndirectedGraphType<unsigned>, typename UndirectedGraphType<unsigned>::InEdgeIterator>
Vertex<UndirectedGraphType<unsigned>>::inEdges(
) const
{
  return typename ::Edges<UndirectedGraphType<unsigned>, typename UndirectedGraphType<unsigned>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, m_graph));
}

/**
 * @brief  Checks if this vertex has an edge to the given vertex.
 */
template <>
bool
Vertex<UndirectedGraphType<unsigned>>::hasEdgeTo(
  const Vertex<UndirectedGraphType<unsigned>>& other
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
template <typename GraphType>
Vertex<GraphType>::Iterator::Iterator(
  const GraphImpl& graph,
  const std::pair<VertexIterator, VertexIterator>& vertices
) : m_graph(graph),
    m_current(vertices.first),
    m_end(vertices.second)
{
}

/**
 * @brief  Increments the iterator.
 */
template <typename GraphType>
typename Vertex<GraphType>::Iterator&
Vertex<GraphType>::Iterator::operator++(
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
bool
Vertex<GraphType>::Iterator::operator==(
  const Vertex<GraphType>::Iterator& that
) const
{
  return (m_current == that.m_current);
}

/**
 * @brief  Checks if the iterator is NOT same as another iterator.
 */
template <typename GraphType>
bool
Vertex<GraphType>::Iterator::operator!=(
  const Vertex<GraphType>::Iterator& that
) const
{
  return (m_current != that.m_current);
}

/**
 * @brief  Returns the vertex that the iterator is currently pointing to. 
 */
template <typename GraphType>
typename ::Vertex<GraphType>
Vertex<GraphType>::Iterator::operator*(
)
{
  return Vertex<GraphType>(m_graph, *m_current);
}

/**
 * @brief  Constructor for the vertex iterator provider.
 *
 * @param graph  Instance of the graph implementation.
 */
template <typename GraphType>
Vertices<GraphType>::Vertices(
  const GraphImpl& graph
) : m_graph(graph)
{
}

/**
 * @brief  Returns the begin iterator over the vertices.
 */
template <>
typename Vertex<UndirectedGraphType<unsigned>>::Iterator
Vertices<UndirectedGraphType<unsigned>>::begin(
) const
{
  return typename Vertex<UndirectedGraphType<unsigned>>::Iterator(m_graph, boost::vertices(m_graph));
}

/**
 * @brief  Returns the end iterator over the vertices.
 */
template <>
typename Vertex<UndirectedGraphType<unsigned>>::Iterator
Vertices<UndirectedGraphType<unsigned>>::end(
) const
{
  VertexIterator end = boost::vertices(m_graph).second;
  return typename Vertex<UndirectedGraphType<unsigned>>::Iterator(m_graph, std::make_pair(end, end));
}

// Explicit instantiation.
template class Vertex<UndirectedGraphType<unsigned>>;
template class Vertices<UndirectedGraphType<unsigned>>;
