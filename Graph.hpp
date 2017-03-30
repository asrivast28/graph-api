/**
 * @file Graph.hpp
 * @brief Declaration of Graph functions.
 */
#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "Edge.hpp"
#include "GraphType.hpp"
#include "Vertex.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief  Enumeration class for different graph file types. 
 */
enum class GraphFileType {
  EDGE_LIST,
  INCIDENCE_MATRIX
};

/**
 * @brief  Class that provides the functionality for reading a graph file. 
 *
 * @tparam FileType      The type of file to be read.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <enum GraphFileType FileType, typename VertexIdType>
class GraphFile {
};

/**
 * @brief  Partial specialization of GraphFile class for FileType = GraphFileType::EDGE_LIST.
 */
template <typename VertexIdType>
class GraphFile<GraphFileType::EDGE_LIST, VertexIdType> {
public:
  GraphFile(const std::string& fileName);

  const std::vector<std::pair<VertexIdType, VertexIdType>>&
  edgeList() const;

  const std::unordered_set<VertexIdType>&
  idSet() const;

private:
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;
  std::unordered_set<VertexIdType> m_idSet;
};

/**
 * @brief  Partial specialization of GraphFile class for FileType = GraphFileType::INCIDENCE_MATRIX.
 */
template <typename VertexIdType>
class GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType> {
public:
  GraphFile(const std::string& fileName);

  const std::vector<std::pair<VertexIdType, VertexIdType>>&
  edgeList() const;

  const std::unordered_set<VertexIdType>&
  idSet() const;

private:
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;
  std::unordered_set<VertexIdType> m_idSet;
};

/**
 * @brief  Skeleton class that outlines the functionality that should be provided by graph class.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <enum GraphFileType FileType, template <typename> class GraphType, typename UnsignedType>
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
  build(const GraphFile<FileType, VertexIdType>&) = 0;
}; // class GraphSkeleton

/**
 * @brief  Class that provides graph functionality.
 *
 * @tparam GraphType     Type of the graph implementation.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <enum GraphFileType FileType, template <typename> class GraphType, typename VertexIdType>
class Graph : public GraphSkeleton<FileType, GraphType, VertexIdType> {
}; // class Graph

/**
 * @brief  Partial specialization of Graph class for GraphType = UndirectedAdjacencyList.
 */
template <enum GraphFileType FileType, typename VertexIdType>
class Graph<FileType, UndirectedAdjacencyList, VertexIdType> : public GraphSkeleton<FileType, UndirectedAdjacencyList, VertexIdType> {
public:
  Graph(const std::string&);

  VertexIterator<UndirectedAdjacencyList, VertexIdType>
  vertices() const;

  VertexIdType
  vertexCount() const;

  typename GraphSkeleton<FileType, UndirectedAdjacencyList, VertexIdType>::Vertex
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
  build(const GraphFile<FileType, VertexIdType>&);

private:
  typename UndirectedAdjacencyList<VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<UndirectedAdjacencyList, VertexIdType>

#endif // GRAPH_HPP_
