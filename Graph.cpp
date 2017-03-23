/**
 * @file Graph.cpp
 * @brief Implementation of Graph functions.
 */
#include "Graph.hpp"

#include "GraphType.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <unordered_set>


/**
 * @brief  Constructor that reads graph in edge list format from the given file.
 *
 * @param fileName  Name of the file from which graph is to be read.
 */
template <>
Graph<UndirectedGraphType<unsigned>>::Graph(
  const std::string& fileName
) : m_graph(),
    m_idVertexMap()
{
  static const std::regex edgePattern("^\\s*\\d+\\s+\\d+\\s*$");

  std::ifstream graphFile(fileName);
  std::string line;
  std::unordered_set<VertexIdType> idSet;
  std::vector<std::pair<VertexIdType, VertexIdType> > edgeList;
  while (std::getline(graphFile, line)) {
    // Read a line if it matches the edge list pattern.
    if (std::regex_search(line, edgePattern)) {
      std::istringstream is(line);
      VertexIdType u, v;
      is >> u >> v;
      edgeList.push_back(std::make_pair(u, v));
      idSet.insert(u);
      idSet.insert(v);
    }
  }
  assert(edgeList.size() > 0);
  m_graph = typename UndirectedGraphType<unsigned>::Impl(idSet.size());
  typename UndirectedGraphType<unsigned>::VertexIterator v = boost::vertices(m_graph).first;
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
 * @brief  Returns an iterator for all the vertices in the graph.
 */
template <typename GraphType>
Vertices<GraphType>
Graph<GraphType>::vertices(
) const
{
  return Vertices<GraphType>(m_graph);
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <>
size_t
Graph<UndirectedGraphType<unsigned>>::vertexCount(
) const
{
  return static_cast<size_t>(boost::num_vertices(m_graph));
}

template <typename GraphType>
typename Graph<GraphType>::Vertex
Graph<GraphType>::getVertexFromId(
  const VertexIdType& v
) const
{
  return Vertex(m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <typename GraphType>
typename GraphType::VertexIdType
Graph<GraphType>::maxVertexId(
) const
{
  using MapPairType = std::pair<VertexIdType, typename GraphType::VertexType>;
  return std::max_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                          [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                         )->first;
}

/**
 * @brief  Returns an iterator for all the edges in the graph.
 */
template <>
Edges<UndirectedGraphType<unsigned>>
Graph<UndirectedGraphType<unsigned>>::edges(
) const
{
  return Edges<UndirectedGraphType<unsigned>>(m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <>
size_t
Graph<UndirectedGraphType<unsigned>>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <typename GraphType>
bool
Graph<GraphType>::edgeExists(
  const Vertex& u,
  const Vertex& v
) const
{
  return u.hasEdgeTo(v);
}

/**
 * @brief  Default destructor.
 */
template <typename GraphType>
Graph<GraphType>::~Graph(
)
{
}

// Explicit instantiation.
template class Graph<UndirectedGraphType<unsigned>>;
