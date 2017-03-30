/**
 * @file Graph.cpp
 * @brief Implementation of Graph functions.
 */
#include "Graph.hpp"


/**
 * @brief  Function that builds an in-memory graph from the given edge list.
 *
 * @param edgeList  List of all the edges in the graph to be built.
 * @param idSet     Set of all the vertex ids in the graph.
 */
template <typename VertexIdType>
void
Graph<UndirectedAdjacencyList, VertexIdType>::build(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet
)
{
  m_graph = typename UndirectedAdjacencyList<VertexIdType>::Impl(idSet.size());
  typename UndirectedAdjacencyList<VertexIdType>::VertexIterator v = boost::vertices(m_graph).first;
  for (const VertexIdType& id : idSet) {
    m_graph[*v].id = id;
    m_idVertexMap.insert(std::make_pair(id, *v));
    ++v;
  }

  for (const std::pair<VertexIdType, VertexIdType>& edge : edgeList) {
    boost::add_edge(m_idVertexMap.at(edge.first), m_idVertexMap.at(edge.second), m_graph);
  }
}

/**
 * @brief  Constructor that reads graph in edge list format from the given file and builds the in-memory graph.
 *
 * @param fileName  Name of the file from which graph is to be read.
 * @param fileType  Type of the file from which graph is to be read.
 */
template <typename VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType>::Graph(
  const std::string& fileName,
  const enum GraphFileType fileType
) : m_graph(),
    m_idVertexMap()
{
  if (fileType == GraphFileType::EDGE_LIST) {
    GraphFile<GraphFileType::EDGE_LIST, VertexIdType> graphFile(fileName);
    build(graphFile.edgeList(), graphFile.idSet());
  }
  else if (fileType == GraphFileType::INCIDENCE_MATRIX) {
    GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType> graphFile(fileName);
    build(graphFile.edgeList(), graphFile.idSet());
  }
}

/**
 * @brief  Returns an iterator for all the vertices in the graph.
 */
template <typename VertexIdType>
VertexIterator<UndirectedAdjacencyList, VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType>::vertices(
) const
{
  return VertexIterator<UndirectedAdjacencyList, VertexIdType>(m_graph);
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <typename VertexIdType>
VertexIdType
Graph<UndirectedAdjacencyList, VertexIdType>::vertexCount(
) const
{
  return static_cast<VertexIdType>(boost::num_vertices(m_graph));
}

template <typename VertexIdType>
typename GraphSkeleton<UndirectedAdjacencyList, VertexIdType>::Vertex
Graph<UndirectedAdjacencyList, VertexIdType>::getVertexFromId(
  const VertexIdType& v
) const
{
  return typename GraphSkeleton<UndirectedAdjacencyList, VertexIdType>::Vertex(m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <typename VertexIdType>
VertexIdType
Graph<UndirectedAdjacencyList, VertexIdType>::maxVertexId(
) const
{
  using MapPairType = std::pair<VertexIdType, typename UndirectedAdjacencyList<VertexIdType>::VertexType>;
  return std::max_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                          [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                         )->first;
}

/**
 * @brief  Returns an iterator for all the edges in the graph.
 */
template <typename VertexIdType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType>::edges(
) const
{
  return EdgeIterator<UndirectedAdjacencyList, VertexIdType>(m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <typename VertexIdType>
size_t
Graph<UndirectedAdjacencyList, VertexIdType>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <typename VertexIdType>
bool
Graph<UndirectedAdjacencyList, VertexIdType>::edgeExists(
  const Vertex<UndirectedAdjacencyList, VertexIdType>& u,
  const Vertex<UndirectedAdjacencyList, VertexIdType>& v
) const
{
  return u.hasEdgeTo(v);
}

/**
 * @brief  Default destructor.
 */
template <typename VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType>::~Graph(
)
{
}

// Explicit instantiation.
template class Graph<UndirectedAdjacencyList, unsigned>;
template class Graph<UndirectedAdjacencyList, size_t>;
