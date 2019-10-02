/**
 * @file GraphType.hpp
 * @brief Declaration of different graph implementation types.
 */
#ifndef GRAPHTYPE_HPP_
#define GRAPHTYPE_HPP_

#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graph_traits.hpp>


/**
 * @brief Generic graph type. Should be specialized for different graph types.
 *
 * @tparam GraphImpl Type of graph implementation.
 * @tparam UnsignedType Unsigned type used for vertex id.
 * @tparam Enable Used for enabling the type for different graph implementations.
 */
template <typename GraphImpl, typename UnsignedType, typename Enable = void>
struct GraphTraits {
  using Impl = void;
  using VertexIdType = void;

  using VertexType = void;
  using VertexProperty = void;
  using VertexIterator = void;
  using AdjacencyIterator = void;
  using EdgeType = void;
  using EdgeIterator = void;
  using InEdgeIterator = void;
  using OutEdgeIterator = void;
};

/**
 * @brief Graph type for Boost graphs.
 *
 * @tparam BoostGraph Type of Boost graph implementation.
 * @tparam UnsignedType Type of vertex id.
 */
template <typename BoostGraph, typename UnsignedType>
struct GraphTraits<BoostGraph, UnsignedType> {
  using Impl = BoostGraph;
  using VertexIdType = UnsignedType;

  using VertexType = typename boost::graph_traits<BoostGraph>::vertex_descriptor;
  using VertexProperty = typename boost::vertex_property<BoostGraph>::type;
  using VertexIterator = typename boost::graph_traits<BoostGraph>::vertex_iterator;
  using AdjacencyIterator = typename boost::graph_traits<BoostGraph>::adjacency_iterator;
  using EdgeType = typename boost::graph_traits<BoostGraph>::edge_descriptor;
  using EdgeIterator = typename boost::graph_traits<BoostGraph>::edge_iterator;
  using InEdgeIterator = typename boost::graph_traits<BoostGraph>::in_edge_iterator;
  using OutEdgeIterator = typename boost::graph_traits<BoostGraph>::out_edge_iterator;
};

/**
 * @brief Boost undirected adjacency list.
 */
template <typename VertexProperties, typename UnsignedType>
using UndirectedAdjacencyList = GraphTraits<boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties>, UnsignedType>;

/**
 * @brief Boost bidirectional adjacency list.
 */
template <typename VertexProperties, typename UnsignedType>
using BidirectionalAdjacencyList = GraphTraits<boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperties>, UnsignedType>;

/**
 * @brief Boost directed adjacency list.
 */
template <typename VertexProperties, typename UnsignedType>
using DirectedAdjacencyList = GraphTraits<boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperties>, UnsignedType>;

/**
 * @brief Boost directed graph.
 */
template <typename VertexProperties, typename UnsignedType>
using DirectedGraph = GraphTraits<boost::directed_graph<VertexProperties>, UnsignedType>;

/**
 * @brief Boost bidirectional CSR graph.
 */
template <typename VertexProperties, typename UnsignedType>
using DirectedCSRGraph = GraphTraits<boost::compressed_sparse_row_graph<boost::directedS, VertexProperties, boost::no_property, boost::no_property, UnsignedType, size_t>, UnsignedType>;

/**
 * @brief Any generic boost graph type.
 */
template <typename GraphImpl, typename UnsignedType>
using GenericBoostGraph = GraphTraits<GraphImpl, UnsignedType>;

/**
 * @brief Used for enabling a template only for boost adjacency list graph type.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename ReturnType = void>
using EnableBoostAdjacencyList = typename std::enable_if<std::is_same<GraphType<Arg, VertexIdType>, UndirectedAdjacencyList<Arg, VertexIdType>>::value |
                                                         std::is_same<GraphType<Arg, VertexIdType>, BidirectionalAdjacencyList<Arg, VertexIdType>>::value |
                                                         std::is_same<GraphType<Arg, VertexIdType>, DirectedAdjacencyList<Arg, VertexIdType>>::value,
                                                         ReturnType
                                                        >::type;

/**
 * @brief Used for enabling a template only for boost CSR graph type.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename ReturnType = void>
using EnableBoostCSR = typename std::enable_if<std::is_same<GraphType<Arg, VertexIdType>, DirectedCSRGraph<Arg, VertexIdType>>::value, ReturnType>::type;

/**
 * @brief Used for enabling a template for all boost graph types.
 */
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType, typename ReturnType = void>
using EnableBoostAll = typename std::enable_if<std::is_same<GraphType<Arg, VertexIdType>, UndirectedAdjacencyList<Arg, VertexIdType>>::value |
                                               std::is_same<GraphType<Arg, VertexIdType>, BidirectionalAdjacencyList<Arg, VertexIdType>>::value |
                                               std::is_same<GraphType<Arg, VertexIdType>, DirectedAdjacencyList<Arg, VertexIdType>>::value |
                                               std::is_same<GraphType<Arg, VertexIdType>, DirectedCSRGraph<Arg, VertexIdType>>::value |
                                               std::is_same<GraphType<Arg, VertexIdType>, GenericBoostGraph<Arg, VertexIdType>>::value,
                                               ReturnType>::type;

#endif // GRAPHTYPE_HPP_
