/**
 * @file Graph.cpp
 * @brief Implementation of Graph functions.
 */
#include "Graph.hpp"

#include <fstream>
#include <regex>
#include <sstream>


/**
 * @brief  Constructor that reads edge list from the given file.
 *
 * @param fileName  Name of the file from which graph is to be read.
 */
template <typename VertexIdType>
GraphFile<GraphFileType::EDGE_LIST, VertexIdType>::GraphFile(
  const std::string& fileName
) : m_edgeList(),
    m_idSet()
{
  static const std::regex edgePattern("^\\s*\\d+\\s+\\d+\\s*$");

  std::ifstream graphFile(fileName);
  std::string line;
  while (std::getline(graphFile, line)) {
    // Read a line if it matches the edge list pattern.
    if (std::regex_search(line, edgePattern)) {
      std::istringstream is(line);
      VertexIdType u, v;
      is >> u >> v;
      m_edgeList.push_back(std::make_pair(u, v));
      m_idSet.insert(u);
      m_idSet.insert(v);
    }
  }
}

/**
 * @brief  Returns the list of all the edges read from the file.
 */
template <typename VertexIdType>
const std::vector<std::pair<VertexIdType, VertexIdType>>&
GraphFile<GraphFileType::EDGE_LIST, VertexIdType>::edgeList(
) const
{
  assert(m_edgeList.size() > 0);
  return m_edgeList;
}

/**
 * @brief  Returns the set of all the vertex ids read from the file. 
 */
template <typename VertexIdType>
const std::unordered_set<VertexIdType>&
GraphFile<GraphFileType::EDGE_LIST, VertexIdType>::idSet(
) const
{
  return m_idSet;
}

/**
 * @brief  Constructor that reads edge list from the given file.
 *
 * @param fileName  Name of the file from which graph is to be read.
 * @param edgeList  List of all the edges read from the file.
 * @param idSet     Set of all the ids read from the file.
 */
template <typename VertexIdType>
GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType>::GraphFile(
  const std::string& fileName
) : m_edgeList(),
    m_idSet()
{
  std::ifstream graphFile(fileName);
  std::string line;
  const std::pair<VertexIdType, VertexIdType> defaultEdge = std::make_pair(0, 0);
  while (std::getline(graphFile, line)) {
    std::istringstream is(line);
    VertexIdType u;
    size_t edgeIndex;
    bool edgeExists;
    is >> u >> edgeIndex >> edgeExists;
    if (!edgeExists) {
      continue;
    }
    if (edgeIndex > m_edgeList.size()) {
      m_edgeList.resize(edgeIndex, defaultEdge);
    }
    std::pair<VertexIdType, VertexIdType>& thisEdge = m_edgeList[edgeIndex - 1];
    if (thisEdge.first == 0) {
      thisEdge.first = u;
    }
    else if (thisEdge.second == 0) {
      thisEdge.second = u;
    }
    else {
      throw std::runtime_error("Edge " + std::to_string(edgeIndex) + " has more than two vertices!");
    }
    m_idSet.insert(u);
  }
}

/**
 * @brief  Returns the list of all the edges read from the file.
 */
template <typename VertexIdType>
const std::vector<std::pair<VertexIdType, VertexIdType>>&
GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType>::edgeList(
) const
{
  assert(m_edgeList.size() > 0);
  return m_edgeList;
}

/**
 * @brief  Returns the set of all the vertex ids read from the file. 
 */
template <typename VertexIdType>
const std::unordered_set<VertexIdType>&
GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType>::idSet(
) const
{
  return m_idSet;
}

/**
 * @brief  Function that builds an in-memory graph from the given edge list.
 *
 * @param edgeList  List of all the edges in the graph to be built.
 * @param idSet     Set of all the vertex ids in the graph.
 */
template <enum GraphFileType FileType, typename VertexIdType>
void
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::build(
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
template <enum GraphFileType FileType, typename VertexIdType>
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::Graph(
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
template <enum GraphFileType FileType, typename VertexIdType>
VertexIterator<UndirectedAdjacencyList, VertexIdType>
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::vertices(
) const
{
  return VertexIterator<UndirectedAdjacencyList, VertexIdType>(m_graph);
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <enum GraphFileType FileType, typename VertexIdType>
VertexIdType
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::vertexCount(
) const
{
  return static_cast<VertexIdType>(boost::num_vertices(m_graph));
}

template <enum GraphFileType FileType, typename VertexIdType>
typename GraphSkeleton<FileType, UndirectedAdjacencyList, VertexIdType>::Vertex
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::getVertexFromId(
  const VertexIdType& v
) const
{
  return typename GraphSkeleton<FileType, UndirectedAdjacencyList, VertexIdType>::Vertex(m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <enum GraphFileType FileType, typename VertexIdType>
VertexIdType
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::maxVertexId(
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
template <enum GraphFileType FileType, typename VertexIdType>
EdgeIterator<UndirectedAdjacencyList, VertexIdType>
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::edges(
) const
{
  return EdgeIterator<UndirectedAdjacencyList, VertexIdType>(m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <enum GraphFileType FileType, typename VertexIdType>
size_t
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <enum GraphFileType FileType, typename VertexIdType>
bool
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::edgeExists(
  const Vertex<UndirectedAdjacencyList, VertexIdType>& u,
  const Vertex<UndirectedAdjacencyList, VertexIdType>& v
) const
{
  return u.hasEdgeTo(v);
}

/**
 * @brief  Default destructor.
 */
template <enum GraphFileType FileType, typename VertexIdType>
Graph<FileType, UndirectedAdjacencyList, VertexIdType>::~Graph(
)
{
}

// Explicit instantiation.
template class Graph<GraphFileType::EDGE_LIST, UndirectedAdjacencyList, unsigned>;
template class Graph<GraphFileType::INCIDENCE_MATRIX, UndirectedAdjacencyList, unsigned>;

template class Graph<GraphFileType::EDGE_LIST, UndirectedAdjacencyList, size_t>;
template class Graph<GraphFileType::INCIDENCE_MATRIX, UndirectedAdjacencyList, size_t>;
