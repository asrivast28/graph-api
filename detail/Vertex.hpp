/**
 * @file Vertex.hpp
 * @brief Details of Vertex and the corresponding iterator functions.
 */
#ifndef DETAIL_VERTEX_HPP_
#define DETAIL_VERTEX_HPP_


/**
 * @brief  Partial specialization of Vertex class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType>
class Vertex<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<VertexProperties, VertexIdType>::Impl;
  using VertexType = typename GraphType<VertexProperties, VertexIdType>::VertexType;
  using IteratorType = typename GraphType<VertexProperties, VertexIdType>::VertexIterator;

public:
  /**
   * @brief  Iterator over vertices of the graph.
   */
  class Iterator : public std::iterator<std::forward_iterator_tag, Vertex>
  {
  public:
    /**
     * @brief  Constructor for iterator over the vertices.
     *
     * @param graph     Instance of the graph implementation.
     * @param vertices  Pair of begin and end iterator implementations over the vertices.
     */
    Iterator(
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
    Iterator&
    operator++(
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
    Iterator
    operator++(
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
    bool
    operator==(
      const Iterator& that
    ) const
    {
      return (m_current == that.m_current);
    }

    /**
     * @brief  Checks if the iterator is NOT same as another iterator.
     */
    bool
    operator!=(
      const Iterator& that
    ) const
    {
      return (m_current != that.m_current);
    }

    /**
     * @brief  Returns the vertex that the iterator is currently pointing to.
     */
    Vertex
    operator*(
    ) const
    {
      return Vertex(m_graph, *m_current);
    }

  private:
    const GraphImpl* const m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  /**
   * @brief  Default constructor for the vertex wrapper.
   */
  Vertex(
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
  Vertex(
    const GraphImpl* const graph,
    VertexType&& vertex
  ) : m_graph(graph),
      m_vertex(vertex)
  {
  }

  /**
   * @brief  Returns the underlying vertex for this wrapper.
   */
  const VertexType&
  operator*() const
  {
    return m_vertex;
  }

  /**
   * @brief  Returns the id of this vertex.
   */
  const VertexProperties&
  properties(
  ) const
  {
    return (*m_graph)[m_vertex];
  }

  /**
   * @brief  Returns the in degree of this vertex.
   */
  VertexIdType
  inDegree(
  ) const
  {
    return boost::in_degree(m_vertex, *m_graph);
  }

  /**
   * @brief  Returns an iterator provider over the edges incident on this vertex.
   */
  EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexProperties, VertexIdType>::InEdgeIterator>
  inEdges(
  ) const
  {
    return EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexProperties, VertexIdType>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, *m_graph));
  }

  /**
   * @brief  Returns the out degree of this vertex.
   */
  VertexIdType
  outDegree(
  ) const
  {
    return boost::out_degree(m_vertex, *m_graph);
  }

  /**
   * @brief  Returns an iterator provider over the outgoing edges from this vertex.
   */
  EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexProperties, VertexIdType>::OutEdgeIterator>
  outEdges(
  ) const
  {
    return EdgeIteratorProvider<GraphType, VertexIdType, typename GraphType<VertexProperties, VertexIdType>::OutEdgeIterator>(m_graph, boost::out_edges(m_vertex, *m_graph));
  }

  /**
   * @brief  Returns a provider for all the simple paths of a given length starting from this vertex.
   */
  SimplePathProvider<GraphType, VertexProperties, VertexIdType>
  simplePaths(
    const VertexIdType& l
  ) const
  {
    return SimplePathProvider<GraphType, VertexProperties, VertexIdType>(*this, l);
  }

  /**
   * @brief  Checks if this vertex has an edge to the given vertex.
   */
  bool
  hasEdgeTo(
    const Vertex& other
  ) const
  {
    return boost::edge(m_vertex, other.m_vertex, *m_graph).second;
  }

private:
  const GraphImpl* m_graph;
  VertexType m_vertex;
}; // class Vertex<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>>

/**
 * @brief  Partial specialization of VertexIteratorProvider class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType>
class VertexIteratorProvider<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
private:
  using IteratorType = typename GraphType<VertexProperties, VertexIdType>::VertexIterator;

public:
  /**
   * @brief  Constructor for the vertex iterator provider.
   *
   * @param graph  Instance of the graph implementation.
   */
  VertexIteratorProvider(
    const typename GraphType<VertexProperties, VertexIdType>::Impl* const graph
  ) : m_graph(graph)
  {
  }

  /**
   * @brief  Returns the begin iterator over the vertices.
   */
  typename Vertex<GraphType, VertexProperties, VertexIdType>::Iterator
  begin(
  ) const
  {
    return typename Vertex<GraphType, VertexProperties, VertexIdType>::Iterator(m_graph, boost::vertices(*m_graph));
  }

  /**
   * @brief  Returns the end iterator over the vertices.
   */
  typename Vertex<GraphType, VertexProperties, VertexIdType>::Iterator
  end(
  ) const
  {
    IteratorType end = boost::vertices(*m_graph).second;
    return typename Vertex<GraphType, VertexProperties, VertexIdType>::Iterator(m_graph, std::make_pair(end, end));
  }

private:
  const typename GraphType<VertexProperties, VertexIdType>::Impl* m_graph;
}; // class VertexIteratorProvider<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>>

#endif // DETAIL_VERTEX_HPP_
