/**
 * @file Edge.hpp
 * @brief Declaration of Edge and the corresponding iterator functions.
 */
#ifndef EDGE_HPP_
#define EDGE_HPP_

#include "GraphType.hpp"


/**
 * @brief Class that provides edge functionality.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam Arg Type of the first argument to the graph implementation template.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename Enable = void>
class Edge {
public:
  template <typename IteratorType>
  class Iterator;
};

/**
 * @brief Class that provides an iterator over edges of the graph.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam Arg Type of the first argument to the graph implementation template.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 * @tparam IteratorType Type of the edge iterator.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename IteratorType = typename GraphType<Arg, VertexIdType>::EdgeIterator, typename Enable = void>
class EdgeIteratorProvider;

#endif // EDGE_HPP_
