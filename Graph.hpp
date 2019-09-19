/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "GraphFile.hpp"
#include "Vertex.hpp"

#include <unordered_map>
#include <vector>


/**
 * @brief Class that provides graph functionality.
 *
 * @tparam GraphType Type of the graph implementation.
 * @tparam VertexProperties Type of the vertex properties.
 * @tparam VertexIdType Unsigned type for storing vertex ids.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType, typename Enable = void>
class Graph;


/**
 * @brief Struct used for providing bundled vertex IDs.
 *
 * @tparam UnsignedType Type of vertex id.
 */
template <typename UnsignedType>
struct VertexInfo {
  UnsignedType id;
};

struct VertexLabel {
  std::string label;
};

#include "detail/GraphFile.hpp"
#include "detail/Edge.hpp"
#include "detail/Vertex.hpp"
#include "detail/SimplePathProvider.hpp"
#include "detail/Graph.hpp"

#endif // GRAPH_HPP_
