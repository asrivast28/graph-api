/**
 * @file Vertex.hpp
 * @brief Details of Vertex and the corresponding iterator functions.
 */
#ifndef DETAIL_VERTEX_HPP_
#define DETAIL_VERTEX_HPP_


/**
 * @brief Partial specialization of Vertex class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType>
class Vertex<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<Arg, VertexIdType>::Impl;
  using VertexType = typename GraphType<Arg, VertexIdType>::VertexType;

public:
  /**
   * @brief Iterator over vertices of the graph.
  *
  * @tparam IteratorType Type of the edge iterator.
  */
  template <typename IteratorType>
  class Iterator : public std::iterator<std::forward_iterator_tag, Vertex>
  {
  public:
    /**
     * @brief Constructor for iterator over the vertices.
     *
     * @param graph Instance of the graph implementation.
     * @param vertices Pair of begin and end iterator implementations over the vertices.
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
     * @brief Increments the iterator using the prefix increment iterator.
     */
    Iterator&
    operator++()
    {
      if (m_current != m_end) {
        ++m_current;
      }
      return *this;
    }

    /**
     * @brief Increments the iterator using the postfix increment iterator.
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
     * @brief Checks if the iterator is same as another iterator.
     */
    bool
    operator==(
      const Iterator& that
    ) const
    {
      return (m_current == that.m_current);
    }

    /**
     * @brief Checks if the iterator is NOT same as another iterator.
     */
    bool
    operator!=(
      const Iterator& that
    ) const
    {
      return (m_current != that.m_current);
    }

    /**
     * @brief Returns the vertex that the iterator is currently pointing to.
     */
    Vertex
    operator*() const
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
   * @brief Default constructor for the vertex wrapper.
   */
  Vertex() : m_graph(nullptr),
      m_vertex(0)
  {
  }

  /**
   * @brief Move constructor for the vertex wrapper.
   *
   * @param graph Instance of the graph implementation.
   * @param vertex Instance of the vertex implementation.
   */
  Vertex(
    const GraphImpl* const graph,
    VertexType&& vertex
  ) : m_graph(graph),
      m_vertex(vertex)
  {
  }

  /**
   * @brief Copy constructor for the vertex wrapper.
   *
   * @param graph Instance of the graph implementation.
   * @param vertex Instance of the vertex implementation.
   */
  Vertex(
    const GraphImpl* const graph,
    const VertexType& vertex
  ) : m_graph(graph),
      m_vertex(vertex)
  {
  }

  /**
   * @brief Returns the underlying vertex for this wrapper.
   */
  const VertexType&
  operator*() const
  {
    return m_vertex;
  }

  /**
   * @brief Compares this vertex with another vertex.
   *        Assumes that the underlying graph is the same.
   */
  bool
  operator==(
    const Vertex& other
  ) const
  {
    return m_vertex == other.m_vertex;
  }

  /**
   * @brief Returns the property of this vertex.
   */
  const typename GraphType<Arg, VertexIdType>::VertexProperty&
  property() const
  {
    return (*m_graph)[m_vertex];
  }

  /**
   * @brief Returns the in degree of this vertex.
   */
  VertexIdType
  inDegree() const
  {
    return boost::in_degree(m_vertex, *m_graph);
  }

  /**
   * @brief Returns an iterator provider over the edges incident on this vertex.
   */
  EdgeIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::InEdgeIterator>
  inEdges() const
  {
    return EdgeIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::InEdgeIterator>(m_graph, boost::in_edges(m_vertex, *m_graph));
  }

  /**
   * @brief Returns the out degree of this vertex.
   */
  VertexIdType
  outDegree() const
  {
    return boost::out_degree(m_vertex, *m_graph);
  }

  /**
   * @brief Returns an iterator provider over the outgoing edges from this vertex.
   */
  EdgeIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::OutEdgeIterator>
  outEdges() const
  {
    return EdgeIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::OutEdgeIterator>(m_graph, boost::out_edges(m_vertex, *m_graph));
  }

  /**
   * @brief Returns an iterator provider over the vertices which are target of outgoing edges from this vertex.
   */
  VertexIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::AdjacencyIterator>
  outNeighbors() const
  {
    return VertexIteratorProvider<GraphType, Arg, VertexIdType, typename GraphType<Arg, VertexIdType>::AdjacencyIterator>(m_graph, boost::adjacent_vertices(m_vertex, *m_graph));
  }

  /**
   * @brief Returns a provider for all the simple paths of a given length starting from this vertex.
   */
  SimplePathProvider<GraphType, Arg, VertexIdType>
  simplePaths(
    const VertexIdType& l
  ) const
  {
    return SimplePathProvider<GraphType, Arg, VertexIdType>(*this, l);
  }

  /**
   * @brief Checks if this vertex has an edge to the given vertex.
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
}; // class Vertex<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>>

/**
 * @brief Partial specialization of VertexIteratorProvider class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename IteratorType>
class VertexIteratorProvider<GraphType, Arg, VertexIdType, IteratorType, EnableBoostAll<GraphType, Arg, VertexIdType>> {
public:
  /**
   * @brief Constructor for the vertex iterator provider.
   *
   * @param graph Instance of the graph implementation.
   */
  VertexIteratorProvider(
    const typename GraphType<Arg, VertexIdType>::Impl* const graph,
    const std::pair<IteratorType, IteratorType>& vertices
  ) : m_graph(graph),
      m_vertices(vertices)
  {
  }

  /**
   * @brief Returns the begin iterator over the vertices.
   */
  typename Vertex<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>
  begin() const
  {
    return typename Vertex<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>(m_graph, m_vertices);
  }

  /**
   * @brief Returns the end iterator over the vertices.
   */
  typename Vertex<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>
  end() const
  {
    IteratorType end = m_vertices.second;
    return typename Vertex<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
  }

private:
  const typename GraphType<Arg, VertexIdType>::Impl* m_graph;
  std::pair<IteratorType, IteratorType> m_vertices;
}; // class VertexIteratorProvider<GraphType, Arg, VertexIdType, IteratorType, EnableBoostAll<GraphType, Arg, VertexIdType>>

#endif // DETAIL_VERTEX_HPP_
