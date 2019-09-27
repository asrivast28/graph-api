/**
 * @file Vertex.hpp
 * @brief Declaration of Vertex and the corresponding iterator functions.
 */
#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "SimplePathProvider.hpp"

#include <iterator>


/**
 * @brief Class that provides vertex functionality.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam Arg Type of the first argument to the graph implementation template.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename Enable = void>
class Vertex {
public:
  class Iterator;
}; // class Vertex

/**
 * @brief Class that provides an iterator over vertices of the graph.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam Arg Type of the first argument to the graph implementation template.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 * @tparam IteratorType Type of the vertex iterator.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename IteratorType = typename GraphType<Arg, VertexIdType>::VertexIterator, typename Enable = void>
class VertexIteratorProvider;

#endif // VERTEX_HPP_
