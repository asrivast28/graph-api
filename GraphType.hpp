/**
 * @file GraphType.hpp
 * @brief Declaration of different graph implementation types.
 */
#ifndef GRAPHTYPE_HPP_
#define GRAPHTYPE_HPP_

#include <boost/graph/compressed_sparse_row_graph.hpp>
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
 * @brief  Boost undirected adjacency list using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using UndirectedAdjacencyList = GraphType<boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexInfo<UnsignedType>>, UnsignedType>;

/**
 * @brief  Boost bidirectional adjacency list using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using BidirectionalAdjacencyList = GraphType<boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo<UnsignedType>>, UnsignedType>;

/**
 * @brief  Boost directed adjacency list using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using DirectedAdjacencyList = GraphType<boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexInfo<UnsignedType>>, UnsignedType>;

/**
 * @brief  Boost bidirectional CSR graph using VertexInfo as vertex bundled properties.
 */
template <typename UnsignedType>
using DirectedCSRGraph = GraphType<boost::compressed_sparse_row_graph<boost::directedS, VertexInfo<UnsignedType>, boost::no_property, boost::no_property, UnsignedType, size_t>, UnsignedType>;

/**
 * @brief  Used for enabling a template only for boost adjacency list graph type.
 */
template <template <typename> class GraphType, typename VertexIdType, typename ReturnType = void>
using EnableBoostAdjacencyList = typename std::enable_if<std::is_same<GraphType<VertexIdType>, UndirectedAdjacencyList<VertexIdType>>::value |
                                                         std::is_same<GraphType<VertexIdType>, BidirectionalAdjacencyList<VertexIdType>>::value |
                                                         std::is_same<GraphType<VertexIdType>, DirectedAdjacencyList<VertexIdType>>::value,
                                                         ReturnType
                                                        >::type;

/**
 * @brief  Used for enabling a template only for boost CSR graph type.
 */
template <template <typename> class GraphType, typename VertexIdType, typename ReturnType = void>
using EnableBoostCSR = typename std::enable_if<std::is_same<GraphType<VertexIdType>, DirectedCSRGraph<VertexIdType>>::value, ReturnType>::type;

/**
 * @brief  Used for enabling a template for all boost graph types.
 */
template <template <typename> class GraphType, typename VertexIdType, typename ReturnType = void>
using EnableBoostAll = typename std::enable_if<std::is_same<decltype(boost::graph_traits<typename GraphType<VertexIdType>::Impl>::null_vertex()), typename GraphType<VertexIdType>::VertexType>::value, ReturnType>::type;

#endif // GRAPHTYPE_HPP_
