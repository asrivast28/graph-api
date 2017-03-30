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
template <typename VertexIdType, enum GraphFileType FileType>
void
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::build(
  const GraphFile<FileType, VertexIdType>& graphFile
)
{
  m_graph = typename UndirectedAdjacencyList<VertexIdType>::Impl(graphFile.idSet().size());
  typename UndirectedAdjacencyList<VertexIdType>::VertexIterator v = boost::vertices(m_graph).first;
  for (const VertexIdType& id : graphFile.idSet()) {
    m_graph[*v].id = id;
    m_idVertexMap.insert(std::make_pair(id, *v));
    ++v;
  }

  for (const std::pair<VertexIdType, VertexIdType>& edge : graphFile.edgeList()) {
    boost::add_edge(m_idVertexMap.at(edge.first), m_idVertexMap.at(edge.second), m_graph);
  }
}

/**
 * @brief  Constructor that reads graph in edge list format from the given file and builds the in-memory graph.
 *
 * @param fileName  Name of the file from which graph is to be read.
 */
template <typename VertexIdType, enum GraphFileType FileType>
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::Graph(
  const std::string& fileName
) : m_graph(),
    m_idVertexMap()
{
  GraphFile<FileType, VertexIdType> graphFile(fileName);
  build(graphFile);
}

/**
 * @brief  Returns an iterator for all the vertices in the graph.
 */
template <typename VertexIdType, enum GraphFileType FileType>
VertexIterator<UndirectedAdjacencyList, VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::vertices(
) const
{
  return VertexIterator<UndirectedAdjacencyList, VertexIdType>(m_graph);
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <typename VertexIdType, enum GraphFileType FileType>
VertexIdType
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::vertexCount(
) const
{
  return static_cast<VertexIdType>(boost::num_vertices(m_graph));
}

template <typename VertexIdType, enum GraphFileType FileType>
typename GraphSkeleton<UndirectedAdjacencyList, VertexIdType, FileType>::Vertex
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::getVertexFromId(
  const VertexIdType& v
) const
{
  return typename GraphSkeleton<UndirectedAdjacencyList, VertexIdType, FileType>::Vertex(m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <typename VertexIdType, enum GraphFileType FileType>
VertexIdType
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::maxVertexId(
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
template <typename VertexIdType, enum GraphFileType FileType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType>
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::edges(
) const
{
  return EdgeIterator<UndirectedAdjacencyList, VertexIdType>(m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <typename VertexIdType, enum GraphFileType FileType>
size_t
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <typename VertexIdType, enum GraphFileType FileType>
bool
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::edgeExists(
  const Vertex<UndirectedAdjacencyList, VertexIdType>& u,
  const Vertex<UndirectedAdjacencyList, VertexIdType>& v
) const
{
  return u.hasEdgeTo(v);
}

/**
 * @brief  Default destructor.
 */
template <typename VertexIdType, enum GraphFileType FileType>
Graph<UndirectedAdjacencyList, VertexIdType, FileType>::~Graph(
)
{
}

// Explicit instantiation.
template class Graph<UndirectedAdjacencyList, unsigned>;
template class Graph<UndirectedAdjacencyList, unsigned, GraphFileType::EDGE_LIST>;
template class Graph<UndirectedAdjacencyList, unsigned, GraphFileType::INCIDENCE_MATRIX>;

template class Graph<UndirectedAdjacencyList, size_t>;
template class Graph<UndirectedAdjacencyList, size_t, GraphFileType::EDGE_LIST>;
template class Graph<UndirectedAdjacencyList, size_t, GraphFileType::INCIDENCE_MATRIX>;
