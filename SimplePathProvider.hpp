/**
 * @file SimplePathProvider.hpp
 * @brief Declaration of simple path provider funcions.
 */
#ifndef SIMPLEPATHPROVIDER_HPP_
#define SIMPLEPATHPROVIDER_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "Vertex.hpp"


/**
 * @brief Class that provides simple path provider functionality.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam VertexProperties Type of the vertex properties.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType, typename Enable = void>
class SimplePathProvider;

#endif // SIMPLEPATHPROVIDER_HPP_
