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
 * @tparam VertexProperties Type of the vertex properties.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType, typename Enable = void>
class Vertex {
public:
  class Iterator;
}; // class Vertex

/**
 * @brief Class that provides an iterator over vertices of the graph.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam VertexProperties Type of the vertex properties.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType, typename Enable = void>
class VertexIteratorProvider;

#endif // VERTEX_HPP_
