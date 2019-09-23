/**
 * @file Edge.hpp
 * @brief Details of Edge and the corresponding iterator functions.
 */
#ifndef DETAIL_EDGE_HPP_
#define DETAIL_EDGE_HPP_

#include "Vertex.hpp"

#include <boost/functional/hash.hpp>


/**
 * @brief  Partial specialization of Edge class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType>
class Edge<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<VertexProperties, VertexIdType>::Impl;
  using EdgeType = typename GraphType<VertexProperties, VertexIdType>::EdgeType;

public:
  /**
   * @brief  Iterator over edges of the graph.
   *
   * @tparam IteratorType  Type of the edge iterator.
   */
  template <typename IteratorType>
  class Iterator : public std::iterator<std::forward_iterator_tag, Edge>
  {
  public:
    /**
     * @brief  Constructor for iterator over the edges.
     *
     * @param graph     Instance of the graph implementation.
     * @param vertices  Pair of begin and end iterator implementations over the edges.
     */
    Iterator(
      const GraphImpl* const graph,
      const std::pair<IteratorType, IteratorType>& edges
    ) : m_graph(graph),
        m_current(edges.first),
        m_end(edges.second)
    {
    }

    /**
     * @brief  Increments the iterator using the prefix increment operator.
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
      const Iterator& other
    ) const
    {
      return (m_current == other.m_current);
    }

    /**
     * @brief  Checks if the iterator is NOT same as another iterator.
     */
    bool
    operator!=(
      const Iterator& other
    ) const
    {
      return (m_current != other.m_current);
    }

    /**
     * @brief  Returns the edge that the iterator is currently pointing to.
     */
    Edge
    operator*(
    ) const
    {
      return Edge<GraphType, VertexProperties, VertexIdType>(m_graph, *m_current);
    }

  private:
    const GraphImpl* const m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  using EdgeIterator = Iterator<typename GraphType<VertexProperties, VertexIdType>::EdgeIterator>;
  using OutEdgeIterator = Iterator<typename GraphType<VertexProperties, VertexIdType>::OutEdgeIterator>;
  using InEdgeIterator = Iterator<typename GraphType<VertexProperties, VertexIdType>::InEdgeIterator>;

public:
  /**
   * @brief  Constructor for the edge wrapper.
   *
   * @param graph  Instance of the graph implementation.
   * @param edge   Instance of the edge implementation.
   */
  Edge(
    const GraphImpl* const graph,
    EdgeType&& edge
  ) : m_graph(graph),
      m_edge(edge)
  {
  }

  /**
   * @brief  Returns the underlying edge for this wrapper.
   */
  const EdgeType&
  operator*() const
  {
    return m_edge;
  }

  /**
   * @brief  Compares this edge with another edge.
   */
  bool
  operator==(const Edge& other) const
  {
    return (m_graph == other.m_graph) && (m_edge == other.m_edge);
  }

  /**
   * @brief  Returns the source vertex of this edge.
   */
  typename ::Vertex<GraphType, VertexProperties, VertexIdType>
  source(
  ) const
  {
    return typename ::Vertex<GraphType, VertexProperties, VertexIdType>(m_graph, boost::source(m_edge, *m_graph));
  }

  /**
   * @brief  Returns the target vertex of this edge.
   */
  typename ::Vertex<GraphType, VertexProperties, VertexIdType>
  target(
  ) const
  {
    return typename ::Vertex<GraphType, VertexProperties, VertexIdType>(m_graph, boost::target(m_edge, *m_graph));
  }

public:
  /**
   * @brief  Hash provider for the edge.
   */
  class Hash {
  public:
    using VertexType = typename GraphType<VertexProperties, VertexIdType>::VertexType;

  public:
    Hash()
      : m_hasher()
    {
    }

    size_t
    operator()(
      const Edge& e
    ) const
    {
      // Use boost::hash for hashing the edge as {source, target} ordered pair
      return m_hasher(std::make_pair(*e.source(), *e.target()));
    }

  private:
    boost::hash<std::pair<VertexType, VertexType>> m_hasher;
  };

private:
  const GraphImpl* m_graph;
  EdgeType m_edge;
}; // class Edge<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>>


/**
 * @brief  Partial specialization of EdgeIteratorProvider class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType, typename IteratorType>
class EdgeIteratorProvider<GraphType, VertexProperties, VertexIdType, IteratorType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
public:
  /**
   * @brief  Constructor for the edge iterator provider.
   *
   * @param graph  Instance of the graph implementation.
   * @param edges  Pair of begin and end iterator implementations over the edges.
   */
  EdgeIteratorProvider(
    const typename GraphType<VertexProperties, VertexIdType>::Impl* const graph,
    const std::pair<IteratorType, IteratorType>& edges
  ) : m_graph(graph),
      m_edges(edges)
  {
  }

  /**
   * @brief  Returns the begin iterator over the edges.
   */
  typename Edge<GraphType, VertexProperties, VertexIdType>::template Iterator<IteratorType>
  begin(
  ) const
  {
    return typename Edge<GraphType, VertexProperties, VertexIdType>::template Iterator<IteratorType>(m_graph, m_edges);
  }

  /**
   * @brief  Returns the end iterator over the edges.
   */
  typename Edge<GraphType, VertexProperties, VertexIdType>::template Iterator<IteratorType>
  end(
  ) const
  {
    IteratorType end = m_edges.second;
    return typename Edge<GraphType, VertexProperties, VertexIdType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
  }

private:
  const typename GraphType<VertexProperties, VertexIdType>::Impl* m_graph;
  std::pair<IteratorType, IteratorType> m_edges;
}; // class EdgeIteratorProvider<GraphType, VertexProperties, VertexIdType, IteratorType, BoostEnable<GraphType, VertexProperties, VertexIdType>>

#endif // DETAIL_EDGE_HPP_
