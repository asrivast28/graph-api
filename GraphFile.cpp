/**
 * @file GraphFile.cpp
 * @brief Implementation of GraphFile functions.
 */
#include "GraphFile.hpp"

#include <cassert>
#include <fstream>
#include <regex>
#include <sstream>


/**
 * @brief  Constructor that reads edge list from the given file.
 *
 * @param fileName  Name of the file from which graph is to be read.
 */
template <typename VertexIdType>
GraphFile<GraphFileType::NONE, VertexIdType>::GraphFile(
  const std::string& 
) : m_edgeList(),
    m_idSet()
{
}

/**
 * @brief  Returns the list of all the edges read from the file.
 */
template <typename VertexIdType>
const std::vector<std::pair<VertexIdType, VertexIdType>>&
GraphFile<GraphFileType::NONE, VertexIdType>::edgeList(
) const
{
  return m_edgeList;
}

/**
 * @brief  Returns the set of all the vertex ids read from the file. 
 */
template <typename VertexIdType>
const std::unordered_set<VertexIdType>&
GraphFile<GraphFileType::NONE, VertexIdType>::idSet(
) const
{
  return m_idSet;
}

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

// Explicit instantiation.
template class GraphFile<GraphFileType::NONE, unsigned>;
template class GraphFile<GraphFileType::NONE, size_t>;

template class GraphFile<GraphFileType::EDGE_LIST, unsigned>;
template class GraphFile<GraphFileType::EDGE_LIST, size_t>;

template class GraphFile<GraphFileType::INCIDENCE_MATRIX, unsigned>;
template class GraphFile<GraphFileType::INCIDENCE_MATRIX, size_t>;