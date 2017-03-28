/**
 * @file Edge.hpp
 * @brief Declaration of Edge and the corresponding iterator functions.
 */
#ifndef EDGE_HPP_
#define EDGE_HPP_

#include "GraphType.hpp"

#include <iterator>


// Forward declaration of Vertex class.
template <template <typename> class GraphType, typename VertexIdType>
class Vertex;

/**
 * @brief  Skeleton class that outlines the functionality that should be provided by edge class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class EdgeSkeleton {
public:
  virtual
  Vertex<GraphType, VertexIdType>
  source() const;

  virtual
  Vertex<GraphType, VertexIdType>
  target() const = 0;
}; // class EdgeSkeleton

/**
 * @brief  Class that provides edge functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Edge : public EdgeSkeleton<GraphType, VertexIdType> {
public:
  template <typename IteratorType>
  class Iterator;
};

/**
 * @brief  Partial specialization of Edge class for GraphType = UndirectedAdjacencyList.
 */
template <typename VertexIdType>
class Edge<UndirectedAdjacencyList, VertexIdType> : public EdgeSkeleton<UndirectedAdjacencyList, VertexIdType> {
private:
  using GraphImpl = typename UndirectedAdjacencyList<VertexIdType>::Impl;
  using EdgeType = typename UndirectedAdjacencyList<VertexIdType>::EdgeType;

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
    Iterator(const GraphImpl&, const std::pair<IteratorType, IteratorType>&);

    Iterator&
    operator++();

    bool
    operator==(const Iterator&) const;

    bool
    operator!=(const Iterator&) const;

    Edge
    operator*();

  private:
    const GraphImpl& m_graph;
    IteratorType m_current;
    const IteratorType m_end;
  }; // class Iterator

public:
  Edge(const GraphImpl&, const EdgeType&);

  Vertex<UndirectedAdjacencyList, VertexIdType>
  source() const;

  Vertex<UndirectedAdjacencyList, VertexIdType>
  target() const;

private:
  const GraphImpl& m_graph;
  const EdgeType m_edge;
}; // class Edge<UndirectedAdjacencyList, VertexIdType>

/**
 * @brief  Skeleton class that outlines the functionality that should be provided by edge iterator provider class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType>
class EdgeIteratorSkeleton {
public:
  virtual
  typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
  begin() const = 0;

  virtual
  typename Edge<GraphType, VertexIdType>::template Iterator<IteratorType>
  end() const = 0;
}; // class EdgeIteratorSkeleton

/**
 * @brief  Class that provides an iterator over edges of the graph.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 * @tparam IteratorType  Type of the edge iterator.
 */
template <template <typename> class GraphType, typename VertexIdType, typename IteratorType = typename GraphType<VertexIdType>::EdgeIterator>
class EdgeIterator : public EdgeIteratorSkeleton<GraphType, VertexIdType, IteratorType> {
}; // class EdgeIterator

/**
 * @brief  Partial specialization of EdgeIterator class for GraphType = UndirectedAdjacencyList.
 */
template <typename VertexIdType, typename IteratorType>
class EdgeIterator<UndirectedAdjacencyList, VertexIdType, IteratorType> : public EdgeIteratorSkeleton<UndirectedAdjacencyList, VertexIdType, IteratorType> {
public:
  EdgeIterator(const typename UndirectedAdjacencyList<VertexIdType>::Impl&, const std::pair<IteratorType, IteratorType>&);

  typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>
  begin() const;

  typename Edge<UndirectedAdjacencyList, VertexIdType>::template Iterator<IteratorType>
  end() const;

private:
  const typename UndirectedAdjacencyList<VertexIdType>::Impl& m_graph;
  const std::pair<IteratorType, IteratorType> m_edges;
}; // class EdgeIterator<UndirectedAdjacencyList, VertexIdType, IteratorType>

#endif // EDGE_HPP_
