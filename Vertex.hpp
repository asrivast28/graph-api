/**
 * @file Vertex.hpp
 * @brief Declaration of Vertex and the corresponding iterator functions.
 */
#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"

#include <iterator>


/**
 * @brief  Class that provides vertex functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename Enable>
class Vertex {
public:
  class Iterator;
}; // class Vertex

/**
 * @brief  Partial specialization of Vertex class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>> {
private:
  using GraphImpl = typename GraphType<VertexIdType>::Impl;
  using VertexType = typename GraphType<VertexIdType>::VertexType;
  using IteratorType = typename GraphType<VertexIdType>::VertexIterator;

public:
  /**
   * @brief  Iterator over vertices of the graph.
   */
  class Iterator : public std::iterator<std::forward_iterator_tag, Vertex>
  {
  public:
    Iterator(const GraphImpl* const, const std::pair<IteratorType, IteratorType>&);

    Iterator&
    operator++();

    bool
    operator==(const Iterator&) const;

    bool
    operator!=(const Iterator&) const;

    Vertex
    operator*();

  private:
    const GraphImpl* const m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  Vertex();

  Vertex(const GraphImpl* const, const VertexType&);

  VertexIdType
  id() const;

  VertexIdType
  inDegree() const;

  typename ::EdgeIterator<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>
  inEdges() const;

  VertexIdType
  outDegree() const;

  typename ::EdgeIterator<GraphType, VertexIdType, typename GraphType<VertexIdType>::OutEdgeIterator>
  outEdges() const;

  bool
  hasEdgeTo(const Vertex<GraphType, VertexIdType>&) const;

private:
  const GraphImpl* m_graph;
  VertexType m_vertex;
}; // class Vertex<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>

/**
 * @brief  Class that provides an iterator over vertices of the graph.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename Enable = void>
class VertexIterator;

/**
 * @brief  Partial specialization of VertexIterator class for Boost graphs.
 */
template <template <typename> class GraphType, typename VertexIdType>
class VertexIterator<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>> {
private:
  using IteratorType = typename GraphType<VertexIdType>::VertexIterator;

public:
  VertexIterator(const typename GraphType<VertexIdType>::Impl* const);

  typename Vertex<GraphType, VertexIdType>::Iterator
  begin() const;

  typename Vertex<GraphType, VertexIdType>::Iterator
  end() const;

private:
  const typename GraphType<VertexIdType>::Impl* const m_graph;
}; // class VertexIterator<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>

#endif // VERTEX_HPP_
