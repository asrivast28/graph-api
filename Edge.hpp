/**
 * @file Edge.hpp
 * @brief Declaration of Edge and the corresponding iterator functions.
 */
#ifndef EDGE_HPP_
#define EDGE_HPP_

#include <iterator>


// Forward declaration of Vertex class.
template <typename GraphType>
class Vertex;

/**
 * @brief  Wrapper around graph edge implementation.
 *
 * @tparam GraphType  Type of the graph implementation.
 */
template <typename GraphType>
class Edge {
private:
  using GraphImpl = typename GraphType::Impl;
  using EdgeType = typename GraphType::EdgeType;

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

  Vertex<GraphType>
  source() const;

  Vertex<GraphType>
  target() const;

private:
  const GraphImpl& m_graph;
  const EdgeType m_edge;
}; // class Edge

/**
 * @brief  Class that provides an iterator over edges of the graph.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam IteratorType  Type of the edge iterator.
 */
template <typename GraphType, typename IteratorType = typename GraphType::EdgeIterator>
class Edges {
private:
  using GraphImpl = typename GraphType::Impl;
  using EdgeIterator = typename GraphType::EdgeIterator;

public:
  Edges(const GraphImpl&, const std::pair<IteratorType, IteratorType>&);

  typename Edge<GraphType>::template Iterator<IteratorType>
  begin() const;

  typename Edge<GraphType>::template Iterator<IteratorType>
  end() const;

private:
  const GraphImpl& m_graph;
  const std::pair<IteratorType, IteratorType> m_edges;
}; // class Edges

#endif // EDGE_HPP_
