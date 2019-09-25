/**
 * @file Graph.hpp
 * @brief Details of Graph functions.
 */
#ifndef DETAIL_GRAPH_HPP_
#define DETAIL_GRAPH_HPP_

#include <boost/graph/copy.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/property_map/property_map.hpp>

#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <map>


/**
 * @brief Function that constructs a boost adjacency list graph from the given edge list.
 *
 * @param edgeList List of all the edges in the graph to be built.
 * @param idSet Set of all the vertex ids in the graph.
 * @param graph Graph constructed by the function.
 * @param idVertexMap Map from the file vertex ids to the corresponding in-memory vertex.
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
 * @brief Function that constructs a boost adjacency compressed sparse row graph from the given edge list.
 *
 * @param edgeList List of all the edges in the graph to be built.
 * @param idSet Set of all the vertex ids in the graph.
 * @param graph Graph constructed by the function.
 * @param idVertexMap Map from the file vertex ids to the corresponding in-memory vertex.
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

template <template <typename, typename> class GraphType, typename VertexIdType>
EnableBoostAll<GraphType, VertexLabel, VertexIdType>
construct(
  const std::vector<std::string>& vertexLabels,
  typename GraphType<VertexLabel, VertexIdType>::Impl& graph,
  std::unordered_map<VertexIdType, typename GraphType<VertexLabel, VertexIdType>::VertexType>& idVertexMap
)
{
  auto id = 0u;
  for (const auto& label: vertexLabels) {
    auto vertex = boost::add_vertex(graph);
    graph[vertex].label = label;
    idVertexMap.insert(std::make_pair(id, vertex));
    ++id;
  }
}

/**
 * @brief Partial specialization of Graph class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType>
class Graph<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
public:
  using Vertex = typename ::Vertex<GraphType, VertexProperties, VertexIdType>;
  using Edge = typename ::Edge<GraphType, VertexProperties, VertexIdType>;
  using VertexType = typename GraphType<VertexProperties, VertexIdType>::VertexType;
  using EdgeType = typename GraphType<VertexProperties, VertexIdType>::EdgeType;

public:
  /**
   * @brief Constructor that reads graph in edge list format from the given file and builds the in-memory graph.
   *
   * @param fileName Name of the file from which graph is to be read.
   * @param fileType Type of the file from which graph is to be read.
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
   * @brief Constructor that crates vertices with given labels, with no edges.
   *
   * @param vertexLabels Labels of all the vertices.
   */
  Graph(
    const std::vector<std::string>& vertexLabels
  ) : m_graph(),
      m_idVertexMap()
  {
    construct<GraphType, VertexIdType>(vertexLabels, m_graph, m_idVertexMap);
  }

  /**
   * @brief Copy constructor.
   */
  Graph(
    const Graph& other
  ) : m_graph(),
      m_idVertexMap(other.m_idVertexMap)
  {
    boost::copy_graph(other.m_graph, m_graph);
  }

  /**
   * @brief Move constructor.
   */
  Graph(
    Graph&& other
  ) : m_graph(other.m_graph),
      m_idVertexMap(std::move(other.m_idVertexMap))
  {
  }

  /**
   * @brief Checks if the graph is directed.
   */
  bool
  isDirected() const
  {
    return boost::is_directed(m_graph);
  }

  /**
   * @brief Returns the number of vertices in the graph.
   */
  VertexIdType
  numVertices() const
  {
    return static_cast<VertexIdType>(boost::num_vertices(m_graph));
  }

  /**
   * @brief Returns an iterator provider for all the vertices in the graph.
   */
  VertexIteratorProvider<GraphType, VertexProperties, VertexIdType>
  vertices() const
  {
    return VertexIteratorProvider<GraphType, VertexProperties, VertexIdType>(&m_graph);
  }

  /**
   * @brief Returns a vector of vertices sorted using the provided comparator.
   *
   * @tparam Comparator Type of the comparator used for sorting.
   * @param comp Comparator function used for sorting.
   */
  template <typename Comparator>
  std::vector<Vertex>
  sorted(
    Comparator&& comp
  ) const
  {
    using VertexType = typename GraphType<VertexProperties, VertexIdType>::VertexType;
    std::vector<Vertex> vertices(boost::num_vertices(m_graph));
    std::transform(boost::vertices(m_graph).first, boost::vertices(m_graph).second, vertices.begin(), [this](const VertexType& u) { return Vertex(&m_graph, u); });
    std::sort(vertices.begin(), vertices.end(), comp);
    return vertices;
  }

  /**
   * @brief Returns the minimum vertex using the provided comparator.
   *
   * @tparam Comparator Type of the comparator used for comparison.
   * @param comp Comparator function.
   */
  template <typename Comparator>
  Vertex
  minVertex(
    Comparator&& comp
  ) const
  {
    using VertexType = typename GraphType<VertexProperties, VertexIdType>::VertexType;
    std::vector<Vertex> vertices(boost::num_vertices(m_graph));
    std::transform(boost::vertices(m_graph).first, boost::vertices(m_graph).second, vertices.begin(), [this](const VertexType& u) { return Vertex(&m_graph, u); });
    return *(std::min_element(vertices.begin(), vertices.end(), comp));
  }

  /**
   * @brief Returns the number of edges in the graph.
   */
  size_t
  numEdges() const
  {
    return static_cast<size_t>(boost::num_edges(m_graph));
  }

  /**
   * @brief Returns an iterator provider for all the edges in the graph.
   */
  EdgeIteratorProvider<GraphType, VertexProperties, VertexIdType>
  edges() const
  {
    return EdgeIteratorProvider<GraphType, VertexProperties, VertexIdType>(&m_graph, boost::edges(m_graph));
  }

  /**
   * @brief Adds an edge between the two vertices, identified by their IDs.
   */
  void
  addEdge(
    const VertexIdType u,
    const VertexIdType v
  )
  {
    boost::add_edge(m_idVertexMap.at(u), m_idVertexMap.at(v), m_graph);
  }

  /**
   * @brief Checks the existence of an edge between the given vertices.
   */
  bool
  edgeExists(
    const Vertex& u,
    const Vertex& v
  ) const
  {
    return u.hasEdgeTo(v);
  }

  /**
   * @brief Checks the existence of an edge between the given vertices, using their IDs.
   */
  bool
  edgeExists(
    const VertexIdType u,
    const VertexIdType v
  ) const
  {
    return boost::edge(m_idVertexMap.at(u), m_idVertexMap.at(v), m_graph).second;
  }

  /**
   * @brief Removes the given edge.
   */
  void
  removeEdge(
    Edge&& e
  )
  {
    boost::remove_edge(*e, m_graph);
  }

  /**
   * @brief Removes the edge from u to v.
   */
  void
  removeEdge(
    const VertexType u,
    const VertexType v
  )
  {
    boost::remove_edge(u, v, m_graph);
  }

  /**
   * @brief Returns set of all the bidirected edges in the graph.
   */
  std::unordered_set<Edge, typename Edge::Hash>
  getBidirectedEdges() const
  {
    std::unordered_set<Edge, typename Edge::Hash> bidirected;
    for (auto e: edges()) {
      if (edgeExists(e.target(), e.source())) {
        bidirected.insert(e);
      }
    }
    return bidirected;
  }

  /**
   * @brief Checks if the graph has any cycles.
   */
  bool
  hasCycles() const
  {
    bool hasCycles = false;
    std::unordered_map<VertexType, size_t> components;
    boost::associative_property_map<decltype(components)> componentMap(components);
    auto numComponents = boost::strong_components(m_graph, componentMap);
    if (numComponents < numVertices()) {
      hasCycles = true;
    }
    return hasCycles;
  }

  /**
   * @brief Writes the graph, with VertexLabel property, to a Graphviz format dot file.
   */
  EnableBoostAdjacencyList<GraphType, VertexLabel, VertexIdType>
  writeGraphviz(
    const std::string& dotFile
  ) const
  {
    std::ofstream stream(dotFile);
    auto vertex_label_writer = [this] (std::ostream& stream, const VertexType& v) { stream << "[label=\"" << m_graph[v].label << "\"]"; };
    auto graph_property_writer = [this] (std::ostream& stream) { stream << "concentrate=true;" << std::endl; };
    boost::write_graphviz(stream, m_graph, vertex_label_writer, boost::default_writer(), graph_property_writer);
  }

  ~Graph()
  {
  }

private:
  typename GraphType<VertexProperties, VertexIdType>::Impl m_graph;
  std::unordered_map<VertexIdType, typename GraphType<VertexProperties, VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>>

#endif // DETAIL_GRAPH_HPP_
