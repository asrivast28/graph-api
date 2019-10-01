/**
 * @file Graph.hpp
 * @brief Details of Graph functions.
 */
#ifndef DETAIL_GRAPH_HPP_
#define DETAIL_GRAPH_HPP_

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/property_map/property_map.hpp>

#include <fstream>
#include <set>
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
template <template <typename, typename> class GraphType, typename Arg, typename VertexIdType>
class Graph<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>> {
public:
  using Vertex = typename ::Vertex<GraphType, Arg, VertexIdType>;
  using Edge = typename ::Edge<GraphType, Arg, VertexIdType>;
  using GraphImpl = typename GraphType<Arg, VertexIdType>::Impl;
  using VertexType = typename GraphType<Arg, VertexIdType>::VertexType;
  using EdgeType = typename GraphType<Arg, VertexIdType>::EdgeType;

private:
  /**
   * @brief Helper class for detecting cycles using DFSVisit.
   */
  class CycleDetector : public boost::default_dfs_visitor {
  public:
    CycleDetector(
      bool& hasCycles
    ) : m_hasCycles(hasCycles) { }

    void
    back_edge(
      const EdgeType,
      const GraphImpl&
    )
    {
      m_hasCycles = true;
    }

  private:
    bool& m_hasCycles;
  };

protected:
  /**
    * @brief Helper class that implements the bidirected edge filter functionality.
   */
  class BidirectedEdgeFilter {
  public:
    BidirectedEdgeFilter(
    ) : m_graph(nullptr),
        m_directed(true)
    {
    }

    BidirectedEdgeFilter(
      const GraphImpl& g,
      const bool filterBidirected = true
    ) : m_graph(&g),
        m_directed(filterBidirected)
    {
    }

    bool
    bidirectedEdge(
      const EdgeType& e
    ) const
    {
      auto source = boost::source(e, *m_graph);
      auto target = boost::target(e, *m_graph);
      return boost::edge(target, source, *m_graph).second;
    }

    bool
    operator()(
      const EdgeType& e
    ) const
    {
      return m_directed ^ bidirectedEdge(e);
    }

  private:
    const GraphImpl* m_graph;
    bool m_directed;
  }; // class BidirectedEdgeFilter

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
   * @brief Constructs the object from an already created implementation and map.
   *
   * @param g The given implementation of the graph.
   * @param idVertexMap The given id to vertex map.
   */
  Graph(
    GraphImpl&& g,
    const std::unordered_map<VertexIdType, typename GraphType<Arg, VertexIdType>::VertexType>& idVertexMap
  ) : m_graph(g),
      m_idVertexMap(idVertexMap)
  {
  }

  /**
   *@brief Returns a wrapper for the given vertex reference.
   */
  Vertex
  wrap(
    const VertexType v
  ) const
  {
    return Vertex(&m_graph, v);
  }

  /**
   *@brief Returns a wrapper for the given edge reference.
   */
  Edge
  wrap(
    const EdgeType e
  ) const
  {
    return Edge(&m_graph, e);
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
  VertexIteratorProvider<GraphType, Arg, VertexIdType>
  vertices() const
  {
    return VertexIteratorProvider<GraphType, Arg, VertexIdType>(&m_graph, boost::vertices(m_graph));
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
    using VertexType = typename GraphType<Arg, VertexIdType>::VertexType;
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
    using VertexType = typename GraphType<Arg, VertexIdType>::VertexType;
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
  EdgeIteratorProvider<GraphType, Arg, VertexIdType>
  edges() const
  {
    return EdgeIteratorProvider<GraphType, Arg, VertexIdType>(&m_graph, boost::edges(m_graph));
  }

  /**
   * @brief Adds an edge between the two vertices, identified by their IDs.
   */
  void
  addEdge(
    const Vertex& source,
    const Vertex& target
  )
  {
    boost::add_edge(*source, *target, m_graph);
  }

  /**
   * @brief Adds an edge between the two vertices.
   */
  void
  addEdge(
    const VertexType source,
    const VertexType target
  )
  {
    boost::add_edge(source, target, m_graph);
  }

  /**
   * @brief Adds an edge between the two vertices, identified by their IDs.
   */
  void
  addEdge(
    const VertexIdType source,
    const VertexIdType target
  )
  {
    boost::add_edge(m_idVertexMap.at(source), m_idVertexMap.at(target), m_graph);
  }

  /**
   * @brief Checks the existence of an edge between the given vertices.
   */
  bool
  edgeExists(
    const Vertex& source,
    const Vertex& target
  ) const
  {
    return source.hasEdgeTo(target);
  }

  /**
   * @brief Checks the existence of an edge between the given vertices, using their IDs.
   */
  bool
  edgeExists(
    const VertexType source,
    const VertexType target
  ) const
  {
    return boost::edge(source, target, m_graph).second;
  }

  /**
   * @brief Checks the existence of an edge between the given vertices, using their IDs.
   */
  bool
  edgeExists(
    const VertexIdType source,
    const VertexIdType target
  ) const
  {
    return boost::edge(m_idVertexMap.at(source), m_idVertexMap.at(target), m_graph).second;
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
   * @brief Removes the edge from source to target.
   */
  void
  removeEdge(
    const Vertex& source,
    const Vertex& target
  )
  {
    boost::remove_edge(*source, *target, m_graph);
  }

  /**
   * @brief Removes the edge from source to target.
   */
  void
  removeEdge(
    const VertexType source,
    const VertexType target
  )
  {
    boost::remove_edge(source, target, m_graph);
  }

  /**
   * @brief Returns set of all the bidirected edges in the graph.
   */
  std::set<Edge>
  bidirectedEdges() const
  {
    std::set<Edge> bidirected;
    for (auto e: edges()) {
      if (edgeExists(e.target(), e.source())) {
        bidirected.insert(e);
      }
    }
    return bidirected;
  }

  /**
   * @brief Returns a filtered view of the current graph with the bidirected edges removed.
   */
  Graph<GenericBoostGraph, boost::filtered_graph<GraphImpl, BidirectedEdgeFilter>, VertexIdType>
  filterBidirectedEdges() const
  {
    BidirectedEdgeFilter bef(m_graph);
    boost::filtered_graph<decltype(m_graph), BidirectedEdgeFilter> fg(m_graph, bef);
    return Graph<GenericBoostGraph, decltype(fg), VertexIdType>(std::move(fg), m_idVertexMap);
  }

  /**
   * @brief Returns a filtered view of the current graph with the directed edges removed.
   */
  Graph<GenericBoostGraph, boost::filtered_graph<GraphImpl, BidirectedEdgeFilter>, VertexIdType>
  filterDirectedEdges() const
  {
    BidirectedEdgeFilter bef(m_graph, false);
    boost::filtered_graph<decltype(m_graph), BidirectedEdgeFilter> fg(m_graph, bef);
    return Graph<GenericBoostGraph, decltype(fg), VertexIdType>(std::move(fg), m_idVertexMap);
  }

  /**
   * @brief Checks if the graph has any cycles.
   */
  bool
  hasCycles() const
  {
    bool hasCycles = false;
    CycleDetector cd(hasCycles);
    boost::depth_first_search(m_graph, boost::visitor(cd));
    return hasCycles;
  }

  /**
   * @brief Checks if the source is part of any cycles in the graph.
   */
  bool
  hasCycles(
    const VertexType source
  ) const
  {
    bool hasCycles = false;
    CycleDetector cd(hasCycles);
    // Color map is required for depth_first_visit
    auto indexMap = boost::get(boost::vertex_index, m_graph);
    auto colorMap = boost::make_vector_property_map<boost::default_color_type>(indexMap);
    // Terminate as soon as a cycle is found
    auto terminateFunc = [&hasCycles] (const VertexType, const GraphImpl&) { return hasCycles; };
    boost::depth_first_visit(m_graph, source, cd, colorMap, terminateFunc);
    return hasCycles;
  }

  /**
   * @brief Writes the graph, with VertexLabel property, to a Graphviz format dot file.
   */
  EnableBoostAll<GraphType, VertexLabel, VertexIdType>
  writeGraphviz(
    const std::string& dotFile
  ) const
  {
    std::ofstream stream(dotFile);
    auto vertex_label_writer = [this] (std::ostream& stream, const VertexType& v) { stream << "[label=\"" << m_graph[v].label << "\"]"; };
    auto graph_property_writer = [this] (std::ostream& stream) { stream << "concentrate=true;" << std::endl; };
    boost::write_graphviz(stream, m_graph, vertex_label_writer, boost::default_writer(), graph_property_writer);
  }

  /**
   * @brief Destructor.
   */
  ~Graph()
  {
  }

private:
  GraphImpl m_graph;
  std::unordered_map<VertexIdType, typename GraphType<Arg, VertexIdType>::VertexType> m_idVertexMap;
}; // class Graph<GraphType, Arg, VertexIdType, EnableBoostAll<GraphType, Arg, VertexIdType>>


#endif // DETAIL_GRAPH_HPP_
