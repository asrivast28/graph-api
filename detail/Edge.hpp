/**
 * @file Edge.hpp
 * @brief Details of Edge and the corresponding iterator functions.
 */
#ifndef DETAIL_EDGE_HPP_
#define DETAIL_EDGE_HPP_

#include "Vertex.hpp"

#include <boost/functional/hash.hpp>


/**
 * @brief Partial specialization of Edge class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType>
class Edge<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<Arg, VertexIdType>::Impl;
  using EdgeType = typename GraphType<Arg, VertexIdType>::EdgeType;

public:
  /**
   * @brief Iterator over edges of the graph.
   *
   * @tparam IteratorType Type of the edge iterator.
   */
  template <typename IteratorType>
  class Iterator : public std::iterator<std::forward_iterator_tag, Edge>
  {
  public:
    /**
     * @brief Constructor for iterator over the edges.
     *
     * @param graph Instance of the graph implementation.
     * @param vertices Pair of begin and end iterator implementations over the edges.
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
     * @brief Increments the iterator using the prefix increment operator.
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
      const Iterator& other
    ) const
    {
      return (m_current == other.m_current);
    }

    /**
     * @brief Checks if the iterator is NOT same as another iterator.
     */
    bool
    operator!=(
      const Iterator& other
    ) const
    {
      return (m_current != other.m_current);
    }

    /**
     * @brief Returns the edge that the iterator is currently pointing to.
     */
    Edge
    operator*() const
    {
      return Edge<GraphType, Arg, VertexIdType>(m_graph, *m_current);
    }

  private:
    const GraphImpl* const m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  using EdgeIterator = Iterator<typename GraphType<Arg, VertexIdType>::EdgeIterator>;
  using OutEdgeIterator = Iterator<typename GraphType<Arg, VertexIdType>::OutEdgeIterator>;
  using InEdgeIterator = Iterator<typename GraphType<Arg, VertexIdType>::InEdgeIterator>;

public:
  /**
   * @brief Default constructor.
   */
  Edge()
    : m_graph(nullptr),
      m_edge()
  {
  }

  /**
   * @brief Move constructor for the edge wrapper.
   *
   * @param graph Instance of the graph implementation.
   * @param edge Instance of the edge implementation.
   */
  Edge(
    const GraphImpl* const graph,
    EdgeType&& edge
  ) : m_graph(graph),
      m_edge(edge)
  {
  }

  /**
   * @brief Copy constructor for the edge wrapper.
   *
   * @param graph Instance of the graph implementation.
   * @param edge Instance of the edge implementation.
   */
  Edge(
    const GraphImpl* const graph,
    const EdgeType& edge
  ) : m_graph(graph),
      m_edge(edge)
  {
  }

  /**
   * @brief Returns the underlying edge for this wrapper.
   */
  const EdgeType&
  operator*() const
  {
    return m_edge;
  }

  /**
   * @brief Compares this edge with another edge.
   *        Assumes that the underlying graph is the same.
   */
  bool
  operator==(
    const Edge& other
  ) const
  {
    return m_edge == other.m_edge;
  }

  /**
   * @brief Partial ordering of the edges, using the vertex ordering.
   */
  bool
  operator<(
    const Edge& other
  ) const
  {
      return (source() < other.source()) ||
            ((source() == other.source()) && (target() < other.target()));
  }

  /**
   * @brief Returns the source vertex of this edge.
   */
  typename ::Vertex<GraphType, Arg, VertexIdType>
  source() const
  {
    return typename ::Vertex<GraphType, Arg, VertexIdType>(m_graph, boost::source(m_edge, *m_graph));
  }

  /**
   * @brief Returns the target vertex of this edge.
   */
  typename ::Vertex<GraphType, Arg, VertexIdType>
  target() const
  {
    return typename ::Vertex<GraphType, Arg, VertexIdType>(m_graph, boost::target(m_edge, *m_graph));
  }

  /**
   * @brief Checks if this edge has an anti-parallel edge.
   */
  bool
  hasAntiParallel() const
  {
    return boost::edge(*target(), *source(), *m_graph).second;
  }

public:
  /**
   * @brief Hash provider for the edge.
   */
  class Hash {
  public:
    using VertexType = typename GraphType<Arg, VertexIdType>::VertexType;

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
}; // class Edge<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>>


/**
 * @brief Partial specialization of EdgeIteratorProvider class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename IteratorType>
class EdgeIteratorProvider<GraphType, Arg, VertexIdType, IteratorType, EnableBoostAll<GraphType, Arg, VertexIdType>> {
public:
  /**
   * @brief Constructor for the edge iterator provider.
   *
   * @param graph Instance of the graph implementation.
   * @param edges Pair of begin and end iterator implementations over the edges.
   */
  EdgeIteratorProvider(
    const typename GraphType<Arg, VertexIdType>::Impl* const graph,
    const std::pair<IteratorType, IteratorType>& edges
  ) : m_graph(graph),
      m_edges(edges)
  {
  }

  /**
   * @brief Returns the begin iterator over the edges.
   */
  typename Edge<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>
  begin() const
  {
    return typename Edge<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>(m_graph, m_edges);
  }

  /**
   * @brief Returns the end iterator over the edges.
   */
  typename Edge<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>
  end() const
  {
    IteratorType end = m_edges.second;
    return typename Edge<GraphType, Arg, VertexIdType>::template Iterator<IteratorType>(m_graph, std::make_pair(end, end));
  }

private:
  const typename GraphType<Arg, VertexIdType>::Impl* m_graph;
  std::pair<IteratorType, IteratorType> m_edges;
}; // class EdgeIteratorProvider<GraphType, Arg, VertexIdType, IteratorType, BoostEnable<GraphType, Arg, VertexIdType>>

#endif // DETAIL_EDGE_HPP_
