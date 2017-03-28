/**
 * @file Vertex.hpp
 * @brief Declaration of Vertex and the corresponding iterator functions.
 */
#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include "GraphType.hpp"

#include <iterator>


// Forward declaration of EdgeIterator class.
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
class EdgeIterator;

/**
 * @brief  Skeleton class that outlines the functionality that should be provided by vertex class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class VertexSkeleton {
public:
  virtual
  VertexIdType
  id() const = 0;

  virtual
  size_t
  inDegree() const;

  virtual
  typename ::EdgeIterator<GraphType, VertexIdType, typename GraphType<VertexIdType>::InEdgeIterator>
  inEdges() const = 0;

  virtual
  bool
  hasEdgeTo(const VertexSkeleton<GraphType, VertexIdType>&) const;
}; // class VertexSkeleton

/**
 * @brief  Class that provides vertex functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Vertex : public VertexSkeleton<GraphType, VertexIdType> {
public:
  class Iterator; 
}; // class Vertex

/**
 * @brief  Partial specialization of Vertex class for GraphType = UndirectedAdjacencyList.
 */
template <typename VertexIdType>
class Vertex<UndirectedAdjacencyList, VertexIdType> : public VertexSkeleton<UndirectedAdjacencyList, VertexIdType> {
private:
  using GraphImpl = typename UndirectedAdjacencyList<VertexIdType>::Impl;
  using VertexType = typename UndirectedAdjacencyList<VertexIdType>::VertexType;
  using IteratorType = typename UndirectedAdjacencyList<VertexIdType>::VertexIterator;

public:
  /**
   * @brief  Iterator over vertices of the graph.
   */
  class Iterator : public std::iterator<std::forward_iterator_tag, Vertex>
  {
  public:
    Iterator(const GraphImpl&, const std::pair<IteratorType, IteratorType>&);

    Iterator&
    operator++();

    bool
    operator==(const Iterator&) const;

    bool
    operator!=(const Iterator&) const;

    Vertex
    operator*();

  private:
    const GraphImpl& m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  Vertex(const GraphImpl&, const VertexType&);

  VertexIdType
  id() const;

  size_t
  inDegree() const;

  typename ::EdgeIterator<UndirectedAdjacencyList, VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::InEdgeIterator>
  inEdges() const;

  bool
  hasEdgeTo(const Vertex<UndirectedAdjacencyList, VertexIdType>&) const;

private:
  const GraphImpl& m_graph;
  const VertexType m_vertex;
}; // class Vertex<UndirectedAdjacencyList, VertexIdType> 

/**
 * @brief  Skeleton class that outlines the functionality that should be provided by vertex iterator provider class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class VertexIteratorSkeleton {
public:
  virtual
  typename Vertex<GraphType, VertexIdType>::Iterator
  begin() const = 0;

  virtual
  typename Vertex<GraphType, VertexIdType>::Iterator
  end() const = 0;
}; // class VertexIteratorSkeleton

/**
 * @brief  Class that provides an iterator over vertices of the graph.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class VertexIterator : public VertexIteratorSkeleton<GraphType, VertexIdType> {
}; // class VertexIterator

/**
 * @brief  Partial specialization of VertexIterator class for GraphType = UndirectedAdjacencyList.
 */
template <typename VertexIdType>
class VertexIterator<UndirectedAdjacencyList, VertexIdType> : public VertexIteratorSkeleton<UndirectedAdjacencyList, VertexIdType> {
private:
  using IteratorType = typename UndirectedAdjacencyList<VertexIdType>::VertexIterator;

public:
  VertexIterator(const typename UndirectedAdjacencyList<VertexIdType>::Impl&);

  typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator
  begin() const;

  typename Vertex<UndirectedAdjacencyList, VertexIdType>::Iterator
  end() const;

private:
  const typename UndirectedAdjacencyList<VertexIdType>::Impl& m_graph;
}; // class VertexIterator<UndirectedAdjacencyList, VertexIdType>

#endif // VERTEX_HPP_
