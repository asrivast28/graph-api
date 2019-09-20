/**
 * @file Graph.hpp
 * @brief Details of Graph functions.
 */
#ifndef DETAIL_GRAPH_HPP_
#define DETAIL_GRAPH_HPP_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/vf2_sub_graph_iso.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include <fstream>


/**
 * @brief  Function that constructs a boost adjacency list graph from the given edge list.
 *
 * @param edgeList     List of all the edges in the graph to be built.
 * @param idSet        Set of all the vertex ids in the graph.
 * @param graph        Graph constructed by the function.
 * @param idVertexMap  Map from the file vertex ids to the corresponding in-memory vertex.
 */
template <template <typename, typename> class GraphType, typename VertexIdType>
EnableBoostAdjacencyList<GraphType, VertexInfo<VertexIdType>, VertexIdType>
construct(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet,
  typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::Impl& graph,
  std::unordered_map<VertexIdType, typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType>& idVertexMap
)
{
  graph = typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::Impl(idSet.size());
  typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexIterator v = boost::vertices(graph).first;
  for (const VertexIdType& id : idSet) {
    graph[*v].id = id;
    idVertexMap.insert(std::make_pair(id, *v));
    ++v;
  }
  auto getVertexTypePair = [&idVertexMap](const std::pair<VertexIdType, VertexIdType>& e) { return std::make_pair(idVertexMap.at(e.first), idVertexMap.at(e.second)); };
  auto edgeBegin = boost::make_transform_iterator(edgeList.begin(), getVertexTypePair);
  auto edgeEnd = boost::make_transform_iterator(edgeList.end(), getVertexTypePair);
  using VertexType = typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType;
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
template <template <typename, typename> class GraphType, typename VertexIdType>
EnableBoostCSR<GraphType, VertexInfo<VertexIdType>, VertexIdType>
construct(
  const std::vector<std::pair<VertexIdType, VertexIdType>>& edgeList,
  const std::unordered_set<VertexIdType>& idSet,
  typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::Impl& graph,
  std::unordered_map<VertexIdType, typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType>& idVertexMap
)
{
  std::vector<std::pair<VertexIdType, VertexIdType>> emptyEdges;
  graph = typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::Impl(boost::edges_are_unsorted, emptyEdges.begin(), emptyEdges.end(), idSet.size());
  typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexIterator v = boost::vertices(graph).first;
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
 * @brief  Partial specialization of Graph class for Boost graphs with VertexInfo as vertex property.
 */
template <template <typename, typename> class GraphType, typename VertexIdType>
class Graph<GraphType, VertexInfo<VertexIdType>, VertexIdType, EnableBoostAll<GraphType, VertexInfo<VertexIdType>, VertexIdType>> {
public:
  using Vertex = typename ::Vertex<GraphType, VertexInfo<VertexIdType>, VertexIdType>;
  using Edge = typename ::Edge<GraphType, VertexInfo<VertexIdType>, VertexIdType>;

public:
  /**
   * @brief  Constructor that reads graph in edge list format from the given file and builds the in-memory graph.
   *
   * @param fileName  Name of the file from which graph is to be read.
   * @param fileType  Type of the file from which graph is to be read.
   */
  Graph(
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
  bool
  isDirected(
  ) const
  {
    return boost::is_directed(m_graph);
  }

  /**
   * @brief  Returns an iterator provider for all the vertices in the graph.
   */
  VertexIteratorProvider<GraphType, VertexInfo<VertexIdType>, VertexIdType>
  vertices(
  ) const
  {
    return VertexIteratorProvider<GraphType, VertexInfo<VertexIdType>, VertexIdType>(&m_graph);
  }

  /**
   * @brief  Returns a vector of vertices sorted using the provided comparator.
   *
   * @tparam Comparator  Type of the comparator used for sorting.
   * @param  comp        Comparator function used for sorting.
   */
  template <typename Comparator>
  std::vector<Vertex>
  sorted(
    Comparator&& comp
  ) const
  {
    using VertexType = typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType;
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
  template <typename Comparator>
  Vertex
  minVertex(
    Comparator&& comp
  ) const
  {
    using VertexType = typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType;
    std::vector<Vertex> vertices(boost::num_vertices(m_graph));
    std::transform(boost::vertices(m_graph).first, boost::vertices(m_graph).second, vertices.begin(), [this](const VertexType& u) { return Vertex(&m_graph, u); });
    return *(std::min_element(vertices.begin(), vertices.end(), comp));
  }


  /**
   * @brief  Returns the number of vertices in the graph.
   */
  VertexIdType
  vertexCount(
  ) const
  {
    return static_cast<VertexIdType>(boost::num_vertices(m_graph));
  }


  Vertex
  getVertexFromId(
    const VertexIdType v
  ) const
  {
    return Vertex(&m_graph, m_idVertexMap.at(v));
  }


  /**
   * @brief  Returns the minimum id of the vertices in the graph.
   */
  VertexIdType
  minVertexId(
  ) const
  {
    using MapPairType = std::pair<VertexIdType, typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType>;
    return std::min_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                            [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                           )->first;
  }


  /**
   * @brief  Returns the maximum id of the vertices in the graph.
   */
  VertexIdType
  maxVertexId(
  ) const
  {
    using MapPairType = std::pair<VertexIdType, typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType>;
    return std::max_element(m_idVertexMap.begin(), m_idVertexMap.end(),
                            [](const MapPairType& a, const MapPairType& b) { return a.first < b.first; }
                           )->first;
  }


  /**
   * @brief  Returns an iterator provider for all the edges in the graph.
   */
  EdgeIteratorProvider<GraphType, VertexInfo<VertexIdType>, VertexIdType>
  edges(
  ) const
  {
    return EdgeIteratorProvider<GraphType, VertexInfo<VertexIdType>, VertexIdType>(&m_graph, boost::edges(m_graph));
  }

  /**
   * @brief  Returns the number of edges in the graph.
   */
  size_t
  edgeCount(
  ) const
  {
    return static_cast<size_t>(boost::num_edges(m_graph));
  }


  /**
   * @brief  Checks the existence of an edge between the given vertices.
   */
  bool
  edgeExists(
    const Vertex& u,
    const Vertex& v
  ) const
  {
    return u.hasEdgeTo(v);
  }

  ~Graph()
  {
  }

private:
  typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename GraphType<VertexInfo<VertexIdType>, VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<GraphType, VertexIdType, EnableBoostAll<GraphType, VertexInfo<VertexIdType>, VertexIdType>>


/**
 * @brief  Partial specialization of Graph class for Boost graphs with VertexLabel as vertex property.
 */
template <template <typename, typename> class GraphType, typename VertexIdType>
class Graph<GraphType, VertexLabel, VertexIdType, EnableBoostAll<GraphType, VertexLabel, VertexIdType>> {
public:
  using Vertex = typename GraphType<VertexLabel, VertexIdType>::VertexType;

public:
  /**
   * @brief  Constructor that crates vertices with given labels, with no edges.
   *
   * @param vertexLabels  Labels of all the vertices.
   */
  Graph(
    const std::vector<std::string>& vertexLabels
  ) : m_graph(),
      m_idVertexMap(vertexLabels.size())
  {
    auto id = 0u;
    for (const auto& label: vertexLabels) {
      auto vertex = boost::add_vertex(m_graph);
      m_graph[vertex].label = label;
      m_idVertexMap[id] = vertex;
      ++id;
    }
  }

  /**
   * @brief  Adds an edge between the two given vertices.
   */
  void
  addEdge(
    const VertexIdType source,
    const VertexIdType dest
  )
  {
    boost::add_edge(m_idVertexMap.at(source), m_idVertexMap.at(dest), m_graph);
  }

  /**
   * @brief  Removes an edge between the two given vertices.
   */
  void
  removeEdge(
    const VertexIdType source,
    const VertexIdType dest
  )
  {
    boost::remove_edge(m_idVertexMap.at(source), m_idVertexMap.at(dest), m_graph);
  }

  /**
   * @brief  Writes the graph to a Graphviz format dot file.
   */
  void
  writeGraphviz(
    const std::string& dotFile
  ) const
  {
    std::ofstream stream(dotFile);
    auto vertex_label_writer = [this] (std::ostream& stream, const Vertex& v) { stream << "[label=\"" << m_graph[v].label << "\"]"; };
    auto graph_property_writer = [this] (std::ostream& stream) { stream << "concentrate=true;" << std::endl; };
    boost::write_graphviz(stream, m_graph, vertex_label_writer, boost::default_writer(), graph_property_writer);
  }

  ~Graph()
  {
  }

private:
  typename GraphType<VertexLabel, VertexIdType>::Impl m_graph;
  std::vector<Vertex> m_idVertexMap;
}; // class Graph<GraphType, UnsignedType, EnableBoostAll<GraphType, VertexLabel, UnsignedType>>

#endif // DETAIL_GRAPH_HPP_
