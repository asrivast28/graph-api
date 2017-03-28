/**
 * @file GraphType.hpp
 * @brief Declaration of different graph implementation types.
 */
#ifndef GRAPHTYPE_HPP_
#define GRAPHTYPE_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>


/**
 * @brief  Generic graph type. Should be specialized for different graph types.
 *
 * @tparam GraphImpl     Type of graph implementation.
 * @tparam UnsignedType  Unsigned type used for vertex id.
 * @tparam Enable        Used for enabling the type for different graph implementations.
 */
template <typename GraphImpl, typename UnsignedType, typename Enable = void>
struct GraphType {
  using Impl = void;
  using VertexIdType = void;

  using VertexType = void;
  using VertexIterator = void;
  using EdgeType = void;
  using EdgeIterator = void;
  using InEdgeIterator = void;
  using OutEdgeIterator = void;
};

/**
 * @brief  Struct used for providing bundled vertex properties.
 *
 * @tparam UnsignedType  Type of vertex id.
 */
template <typename UnsignedType>
struct VertexInfo {
  UnsignedType id;
};

/**
 * @brief  Typedef for Boost undirected graph using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using UndirectedGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexInfo<UnsignedType>>;

/**
 * @brief  Graph type for Boost graphs.
 *
 * @tparam BoostGraph    Type of Boost graph implementation.
 * @tparam UnsignedType  Type of vertex id.
 */
template <typename BoostGraph, typename UnsignedType>
struct GraphType<BoostGraph, UnsignedType, typename std::enable_if<std::is_same<typename boost::vertex_bundle_type<BoostGraph>::type, VertexInfo<UnsignedType>>::value>::type> {
  using Impl = BoostGraph;
  using VertexIdType = UnsignedType;

  using VertexType = typename boost::graph_traits<BoostGraph>::vertex_descriptor;
  using VertexIterator = typename boost::graph_traits<BoostGraph>::vertex_iterator ;
  using EdgeType = typename boost::graph_traits<BoostGraph>::edge_descriptor;
  using EdgeIterator = typename boost::graph_traits<BoostGraph>::edge_iterator;
  using InEdgeIterator = typename boost::graph_traits<BoostGraph>::in_edge_iterator;
  using OutEdgeIterator = typename boost::graph_traits<BoostGraph>::out_edge_iterator;
};

/**
 * @brief  Boost undirected graph type using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using UndirectedAdjacencyList = GraphType<UndirectedGraph<UnsignedType>, UnsignedType>;

#endif // GRAPHTYPE_HPP_
