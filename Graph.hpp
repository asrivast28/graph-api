/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "GraphFile.hpp"
#include "GraphType.hpp"
#include "Vertex.hpp"

#include <unordered_map>
#include <vector>


/**
 * @brief  Skeleton class that outlines the functionality that should be provided by graph class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename UnsignedType>
class GraphSkeleton {
public:
  using VertexIdType = UnsignedType;

  using Vertex = typename ::Vertex<GraphType, VertexIdType>;
  using Edge = typename ::Edge<GraphType, VertexIdType>;

public:
  virtual
  VertexIterator<GraphType, VertexIdType>
  vertices() const = 0;

  virtual
  VertexIdType
  vertexCount() const = 0;

  virtual
  Vertex
  getVertexFromId(const VertexIdType&) const = 0;

  virtual
  VertexIdType
  maxVertexId() const = 0;

  virtual
  EdgeIterator<GraphType, VertexIdType>
  edges() const = 0;

  virtual
  size_t
  edgeCount() const = 0;

  virtual
  bool
  edgeExists(const Vertex&, const Vertex&) const = 0;

protected:
  virtual
  void
  build(const std::vector<std::pair<VertexIdType, VertexIdType>>&, const std::unordered_set<VertexIdType>&) = 0;
}; // class GraphSkeleton

/**
 * @brief  Class that provides graph functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <template <typename> class GraphType, typename VertexIdType>
class Graph : public GraphSkeleton<GraphType, VertexIdType> {
}; // class Graph

/**
 * @brief  Partial specialization of Graph class for GraphType = UndirectedAdjacencyList.
 */
template <typename VertexIdType>
class Graph<UndirectedAdjacencyList, VertexIdType> : public GraphSkeleton<UndirectedAdjacencyList, VertexIdType> {
public:
  Graph(const std::string&, const enum GraphFileType);

  VertexIterator<UndirectedAdjacencyList, VertexIdType>
  vertices() const;

  VertexIdType
  vertexCount() const;

  typename GraphSkeleton<UndirectedAdjacencyList, VertexIdType>::Vertex
  getVertexFromId(const VertexIdType&) const;

  VertexIdType
  maxVertexId() const;

  EdgeIterator<UndirectedAdjacencyList, VertexIdType>
  edges() const;

  size_t
  edgeCount() const;

  bool
  edgeExists(const Vertex<UndirectedAdjacencyList, VertexIdType>&, const Vertex<UndirectedAdjacencyList, VertexIdType>&) const;

  ~Graph();

private:
  void
  build(const std::vector<std::pair<VertexIdType, VertexIdType>>&, const std::unordered_set<VertexIdType>&);

private:
  typename UndirectedAdjacencyList<VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<UndirectedAdjacencyList, VertexIdType>

#endif // GRAPH_HPP_
