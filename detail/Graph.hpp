/**
 * @file Graph.hpp
 * @brief Details of Graph functions.
 */
#ifndef DETAIL_GRAPH_HPP_
#define DETAIL_GRAPH_HPP_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/vf2_sub_graph_iso.hpp>
#include <boost/iterator/transform_iterator.hpp>


/**
 * @brief  Function that constructs a boost adjacency list graph from the given edge list.
 *
 * @param edgeList     List of all the edges in the graph to be built.
 * @param idSet        Set of all the vertex ids in the graph.
 * @param graph        Graph constructed by the function.
 * @param idVertexMap  Map from the file vertex ids to the corresponding in-memory vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
EnableBoostAdjacencyList<GraphType, VertexIdType>
construct(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet,
  typename GraphType<VertexIdType>::Impl& graph,
  std::unordered_map<VertexIdType, typename GraphType<VertexIdType>::VertexType>& idVertexMap
)
{
  graph = typename GraphType<VertexIdType>::Impl(idSet.size());
  typename GraphType<VertexIdType>::VertexIterator v = boost::vertices(graph).first;
  for (const VertexIdType& id : idSet) {
    graph[*v].id = id;
    idVertexMap.insert(std::make_pair(id, *v));
    ++v;
  }
  auto getVertexTypePair = [&idVertexMap](const std::pair<VertexIdType, VertexIdType>& e) { return std::make_pair(idVertexMap.at(e.first), idVertexMap.at(e.second)); };
  auto edgeBegin = boost::make_transform_iterator(edgeList.begin(), getVertexTypePair);
  auto edgeEnd = boost::make_transform_iterator(edgeList.end(), getVertexTypePair);
  using VertexType = typename GraphType<VertexIdType>::VertexType;
  for (const std::pair<VertexType, VertexType>& edge : boost::make_iterator_range(edgeBegin, edgeEnd)) {
    boost::add_edge(edge.first, edge.second, graph);
  }
}

/**
 * @brief  Function that constructs a boost adjacency compressed sparse row graph from the given edge list.
 *
 * @param edgeList     List of all the edges in the graph to be built.
 * @param idSet        Set of all the vertex ids in the graph.
 * @param graph        Graph constructed by the function.
 * @param idVertexMap  Map from the file vertex ids to the corresponding in-memory vertex.
 */
template <template <typename> class GraphType, typename VertexIdType>
EnableBoostCSR<GraphType, VertexIdType>
construct(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet,
  typename GraphType<VertexIdType>::Impl& graph,
  std::unordered_map<VertexIdType, typename GraphType<VertexIdType>::VertexType>& idVertexMap
)
{
  std::vector<std::pair<VertexIdType, VertexIdType>> emptyEdges;
  graph = typename GraphType<VertexIdType>::Impl(boost::edges_are_unsorted, emptyEdges.begin(), emptyEdges.end(), idSet.size());
  typename GraphType<VertexIdType>::VertexIterator v = boost::vertices(graph).first;
  for (const VertexIdType& id : idSet) {
    graph[*v].id = id;
    idVertexMap.insert(std::make_pair(id, *v));
    ++v;
  }
  auto getVertexTypePair = [&idVertexMap](const std::pair<VertexIdType, VertexIdType>& e) { return std::make_pair(idVertexMap.at(e.first), idVertexMap.at(e.second)); };
  auto edgeBegin = boost::make_transform_iterator(edgeList.begin(), getVertexTypePair);
  auto edgeEnd = boost::make_transform_iterator(edgeList.end(), getVertexTypePair);
  boost::add_edges(edgeBegin, edgeEnd, graph);
}

/**
 * @brief  Constructor that reads graph in edge list format from the given file and builds the in-memory graph.
 *
 * @param fileName  Name of the file from which graph is to be read.
 * @param fileType  Type of the file from which graph is to be read.
 */
template <template <typename> class GraphType, typename VertexIdType>
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Graph(
  const std::string& fileName,
  const enum GraphFileType fileType
) : m_graph(),
    m_idVertexMap()
{
  if (fileType == GraphFileType::EDGE_LIST) {
    GraphFile<GraphFileType::EDGE_LIST, VertexIdType> graphFile(fileName);
    construct<GraphType, VertexIdType>(graphFile.edgeList(), graphFile.idSet(), m_graph, m_idVertexMap);
  }
  else if (fileType == GraphFileType::INCIDENCE_MATRIX) {
    GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType> graphFile(fileName);
    construct<GraphType, VertexIdType>(graphFile.edgeList(), graphFile.idSet(), m_graph, m_idVertexMap);
  }
  else if (fileType == GraphFileType::ARG_DATABASE) {
    GraphFile<GraphFileType::ARG_DATABASE, VertexIdType> graphFile(fileName);
    construct<GraphType, VertexIdType>(graphFile.edgeList(), graphFile.idSet(), m_graph, m_idVertexMap);
  }
  else if (fileType == GraphFileType::FHCP) {
    GraphFile<GraphFileType::FHCP, VertexIdType> graphFile(fileName);
    construct<GraphType, VertexIdType>(graphFile.edgeList(), graphFile.idSet(), m_graph, m_idVertexMap);
  }
  else {
    throw std::runtime_error("Graph file type not found!");
  }
}

/**
 * @brief  Checks if the graph is directed.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::isDirected(
) const
{
  return boost::is_directed(m_graph);
}

/**
 * @brief  Returns an iterator for all the vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIterator<GraphType, VertexIdType>
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::vertices(
) const
{
  return VertexIterator<GraphType, VertexIdType>(&m_graph);
}

/**
 * @brief  Returns a vector of vertices sorted using the provided comparator.
 *
 * @tparam Comparator  Type of the comparator used for sorting.
 * @param  comp        Comparator function used for sorting.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename Comparator>
std::vector<typename Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Vertex>
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::sorted(
  Comparator&& comp
) const
{
  using VertexType = typename GraphType<VertexIdType>::VertexType;
  std::vector<Vertex> vertices(boost::num_vertices(m_graph));
  std::transform(boost::vertices(m_graph).first, boost::vertices(m_graph).second, vertices.begin(), [this](const VertexType& u) { return Vertex(&m_graph, u); });
  std::sort(vertices.begin(), vertices.end(), comp);
  return vertices;
}

/**
 * @brief  Returns the minimum vertex using the provided comparator.
 *
 * @tparam Comparator  Type of the comparator used for comparison.
 * @param  comp        Comparator function.
 */
template <template <typename> class GraphType, typename VertexIdType>
template <typename Comparator>
typename Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Vertex
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::minElement(
  Comparator&& comp
) const
{
  using VertexType = typename GraphType<VertexIdType>::VertexType;
  std::vector<Vertex> vertices(boost::num_vertices(m_graph));
  std::transform(boost::vertices(m_graph).first, boost::vertices(m_graph).second, vertices.begin(), [this](const VertexType& u) { return Vertex(&m_graph, u); });
  return *(std::min_element(vertices.begin(), vertices.end(), comp));
}

/**
 * @brief  Returns the number of vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::vertexCount(
) const
{
  return static_cast<VertexIdType>(boost::num_vertices(m_graph));
}

template <template <typename> class GraphType, typename VertexIdType>
typename Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::Vertex
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::getVertexFromId(
  const VertexIdType& v
) const
{
  return Vertex(&m_graph, m_idVertexMap.at(v));
}

/**
 * @brief  Returns the minimum id of the vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::minVertexId(
) const
{
  using MapPairType = std::pair<VertexIdType, typename GraphType<VertexIdType>::VertexType>;
  return std::min_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                          [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                         )->first;
}

/**
 * @brief  Returns the maximum id of the vertices in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
VertexIdType
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::maxVertexId(
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
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::edges(
) const
{
  return EdgeIterator<GraphType, VertexIdType>(&m_graph, boost::edges(m_graph));
}

/**
 * @brief  Returns the number of edges in the graph.
 */
template <template <typename> class GraphType, typename VertexIdType>
size_t
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::edgeCount(
) const
{
  return static_cast<size_t>(boost::num_edges(m_graph));
}

/**
 * @brief  Checks the existence of an edge between the given vertices.
 */
template <template <typename> class GraphType, typename VertexIdType>
bool
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::edgeExists(
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
Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexIdType>>::~Graph(
)
{
}

#endif // DETAIL_GRAPH_HPP_
