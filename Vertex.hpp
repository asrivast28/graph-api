/**
 * @file Vertex.hpp
 * @brief Declaration of Vertex and the corresponding iterator functions.
 */
#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <iterator>


// Forward declaration of Edges class.
template <typename GraphType, typename IteratorType>
class Edges;

/**
 * @brief  Wrapper around graph vertex implementation.
 *
 * @tparam GraphType  Type of the graph implementation.
 */
template <typename GraphType>
class Vertex {
private:
  using GraphImpl = typename GraphType::Impl;
  using VertexIdType = typename GraphType::VertexIdType;
  using VertexType = typename GraphType::VertexType;
  using VertexIterator = typename GraphType::VertexIterator;

public:
  /**
   * @brief  Iterator over vertices of the graph.
   */
  class Iterator : public std::iterator<std::forward_iterator_tag, Vertex>
  {
  public:
    Iterator(const GraphImpl&, const std::pair<VertexIterator, VertexIterator>&);

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
    VertexIterator m_current;
    const VertexIterator m_end;
  }; // class Iterator

public:
  Vertex(const GraphImpl&, const VertexType&);

  VertexIdType
  id() const;

  size_t
  inDegree() const;

  typename ::Edges<GraphType, typename GraphType::InEdgeIterator>
  inEdges() const;

  bool
  hasEdgeTo(const Vertex<GraphType>&) const;

private:
  const GraphImpl& m_graph;
  const VertexType m_vertex;
}; // class Vertex

/**
 * @brief  Class that provides an iterator over vertices of the graph.
 *
 * @tparam GraphType  Type of the graph implementation.
 * */
template <typename GraphType>
class Vertices {
private:
  using GraphImpl = typename GraphType::Impl;
  using VertexIterator = typename GraphType::VertexIterator;

public:
  Vertices(const GraphImpl&);

  typename Vertex<GraphType>::Iterator
  begin() const;

  typename Vertex<GraphType>::Iterator
  end() const;

private:
  const GraphImpl& m_graph;
};

#endif // VERTEX_HPP_
