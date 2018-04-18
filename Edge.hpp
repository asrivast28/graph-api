/**
 * @file Edge.hpp
 * @brief Declaration of Edge and the corresponding iterator functions.
 */
#ifndef EDGE_HPP_
#define EDGE_HPP_

#include "GraphType.hpp"

#include <iterator>


// Forward declaration of Vertex class.
template <template <typename> class GraphType, typename VertexIdType, typename Enable = void>
class Vertex;

/**
 * @brief  Class that provides edge functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename Enable = void>
class Edge {
public:
  template <typename IteratorType>
  class Iterator;
};

/**
 * @brief  Partial specialization of Edge class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Edge<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<VertexIdType>::Impl;
  using EdgeType = typename GraphType<VertexIdType>::EdgeType;

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
    Iterator(const GraphImpl* const, const std::pair<IteratorType, IteratorType>&);

    Iterator&
    operator++();

    Iterator
    operator++(int);

    bool
    operator==(const Iterator&) const;

    bool
    operator!=(const Iterator&) const;

    Edge
    operator*() const;

  private:
    const GraphImpl* const m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  using EdgeIterator = Iterator<typename GraphType<VertexIdType>::EdgeIterator>;
  using OutEdgeIterator = Iterator<typename GraphType<VertexIdType>::OutEdgeIterator>;
  using InEdgeIterator = Iterator<typename GraphType<VertexIdType>::InEdgeIterator>;

public:
  Edge(const GraphImpl* const, const EdgeType&);

  Vertex<GraphType, VertexIdType>
  source() const;

  Vertex<GraphType, VertexIdType>
  target() const;

private:
  const GraphImpl* m_graph;
  EdgeType m_edge;
}; // class Edge<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>

/**
 * @brief  Class that provides an iterator over edges of the graph.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 * @tparam IteratorType  Type of the edge iterator.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType = typename GraphType<VertexIdType>::EdgeIterator, typename Enable = void>
class EdgeIteratorProvider;

/**
 * @brief  Partial specialization of EdgeIteratorProvider class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
class EdgeIteratorProvider<GraphType, VertexIdType, IteratorType, EnableBoostAll<GraphType, VertexIdType>> {
public:
  EdgeIteratorProvider(const typename GraphType<VertexIdType>::Impl* const, const std::pair<IteratorType, IteratorType>&);

  typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
  begin() const;

  typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
  end() const;

private:
  const typename GraphType<VertexIdType>::Impl* const m_graph;
  const std::pair<IteratorType, IteratorType> m_edges;
}; // class EdgeIteratorProvider<GraphType, VertexIdType, IteratorType, BoostEnable<GraphType, VertexIdType>>

#endif // EDGE_HPP_
