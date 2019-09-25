/**
 * @file GraphFile.hpp
 * @brief Details of GraphFile functions.
 */
#ifndef DETAIL_GRAPHFILE_HPP_
#define DETAIL_GRAPHFILE_HPP_

#include <cassert>
#include <fstream>
#include <regex>
#include <sstream>

#define IMPL_GRAPH_FILE(FileType)\
/**\
 * @brief Returns the list of all the edges read from the file.\
 */\
template <typename VertexIdType>\
const std::vector<std::pair<VertexIdType, VertexIdType>>&\
GraphFile<FileType, VertexIdType>::edgeList(\
) const\
{\
  assert(m_edgeList.size() > 0);\
  return m_edgeList;\
}\
\
/**\
 * @brief Returns the set of all the vertex ids read from the file.\
G */\
template <typename VertexIdType>\
const std::unordered_set<VertexIdType>&\
GraphFile<FileType, VertexIdType>::idSet(\
) const\
{\
  return m_idSet;\
}


/**
 * @brief Constructor that reads edge list from the given file.
 *
 * @param fileName Name of the file from which graph is to be read.
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

IMPL_GRAPH_FILE(GraphFileType::EDGE_LIST)

/**
 * @brief Constructor that reads edge list from the given file.
 *
 * @param fileName Name of the file from which graph is to be read.
 * @param edgeList List of all the edges read from the file.
 * @param idSet Set of all the ids read from the file.
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

IMPL_GRAPH_FILE(GraphFileType::INCIDENCE_MATRIX)


template <unsigned WordSize, typename TargetType>
TargetType
read_word(
  std::ifstream& fileStream
)
{
  TargetType var;
  char buf[sizeof(TargetType)] = {0};
  fileStream.read(buf, WordSize);
  memcpy(&var, buf, sizeof(TargetType));
  return var;
}

/**
 * @brief Constructor that reads edge list from the given file.
 *
 * @param fileName Name of the file from which graph is to be read.
 * @param edgeList List of all the edges read from the file.
 * @param idSet Set of all the ids read from the file.
 */
template <typename VertexIdType>
GraphFile<GraphFileType::ARG_DATABASE, VertexIdType>::GraphFile(
  const std::string& fileName
) : m_edgeList(),
    m_idSet()
{
  std::ifstream graphFile(fileName, std::ios::in | std::ios::binary);
  VertexIdType nodeCount = read_word<2, VertexIdType>(graphFile);
  size_t edgeCount = 0;
  for (VertexIdType u = 1; u <= nodeCount; ++u) {
    size_t thisEdgeCount = read_word<2, size_t>(graphFile);
    m_edgeList.resize(edgeCount + thisEdgeCount, std::make_pair(u, 0));
    for (size_t edge = edgeCount; edge < edgeCount + thisEdgeCount; ++edge) {
      VertexIdType v = read_word<2, VertexIdType>(graphFile);
      m_edgeList[edge].second = v + 1;
      m_idSet.insert(v + 1);
    }
    m_idSet.insert(u);
    edgeCount += thisEdgeCount;
  }
}

IMPL_GRAPH_FILE(GraphFileType::ARG_DATABASE)


/**
 * @brief Constructor that reads edge list from the given file.
 *
 * @param fileName Name of the file from which graph is to be read.
 * @param edgeList List of all the edges read from the file.
 * @param idSet Set of all the ids read from the file.
 */
template <typename VertexIdType>
GraphFile<GraphFileType::FHCP, VertexIdType>::GraphFile(
  const std::string& fileName
) : m_edgeList(),
    m_idSet()
{
  std::ifstream graphFile(fileName);
  std::string line;
  while (std::getline(graphFile, line)) {
    if (line.find("EDGE_DATA_SECTION") != std::string::npos) {
      break;
    }
  }
  while (std::getline(graphFile, line)) {
    if (line.find("-1") != std::string::npos) {
      break;
    }
    std::istringstream is(line);
    VertexIdType u, v;
    is >> u >> v;
    m_edgeList.push_back(std::make_pair(u, v));
    m_idSet.insert(u);
    m_idSet.insert(v);
  }
}

IMPL_GRAPH_FILE(GraphFileType::FHCP)


#endif // DETAIL_GRAPHFILE_HPP_
