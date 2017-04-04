/**
 * @file Graph.hpp
 * @brief Details of Graph functions.
 */
#ifndef DETAIL_GRAPH_HPP_
#define DETAIL_GRAPH_HPP_


/**
 * @brief  Function that builds an in-memory graph from the given edge list.
 *
 * @param edgeList  List of all the edges in the graph to be built.
 * @param idSet     Set of all the vertex ids in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
void
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::build(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet
)
{
  m_graph = typename GraphType<VertexIdType>::Impl(idSet.size());
  typename GraphType<VertexIdType>::VertexIterator v = boost::vertices(m_graph).first;
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
template <template <typename> class GraphType, typename VertexIdType>
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Graph(
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
  else if (fileType == GraphFileType::ARG_DATABASE) {
    GraphFile<GraphFileType::ARG_DATABASE, VertexIdType> graphFile(fileName);
    build(graphFile.edgeList(), graphFile.idSet());
  }
}

/**
 * @brief  Returns an iterator for all the vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIterator<GraphType, VertexIdType>
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::vertices(
) const
{
  return VertexIterator<GraphType, VertexIdType>(m_graph);
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::vertexCount(
) const
{
  return static_cast<VertexIdType>(boost::num_vertices(m_graph));
}

template <template <typename> class GraphType, typename VertexIdType>
typename Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::Vertex
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::getVertexFromId(
  const VertexIdType& v
) const
{
  return Vertex(m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::maxVertexId(
) const
{
  using MapPairType = std::pair<VertexIdType, typename GraphType<VertexIdType>::VertexType>;
  return std::max_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                          [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                         )->first;
}

/**
 * @brief  Returns an iterator for all the edges in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
EdgeIterator<GraphType, VertexIdType>
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::edges(
) const
{
  return EdgeIterator<GraphType, VertexIdType>(m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
size_t
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::edgeExists(
  const Vertex& u,
  const Vertex& v
) const
{
  return u.hasEdgeTo(v);
}

/**
 * @brief  Default destructor.
 */
template <template <typename> class GraphType, typename VertexIdType>
Graph<GraphType, VertexIdType, EnableBoost<GraphType, VertexIdType>>::~Graph(
)
{
}

#endif // DETAIL_GRAPH_HPP_
