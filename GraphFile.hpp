/**
 * @file GraphFile.hpp
 * @brief Declaration of GraphFile functions.
 */
#ifndef GRAPHFILE_HPP_
#define GRAPHFILE_HPP_

#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief  Enumeration class for different graph file types.
 */
enum class GraphFileType {
  EDGE_LIST,
  INCIDENCE_MATRIX,
  ARG_DATABASE
};

/**
 * @brief  Class that provides the functionality for reading a graph file.
 *
 * @tparam FileType      The type of file to be read.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <enum GraphFileType FileType, typename VertexIdType>
class GraphFile;

/**
 * @brief  Partial specialization of GraphFile class for FileType = GraphFileType::EDGE_LIST.
 */
template <typename VertexIdType>
class GraphFile<GraphFileType::EDGE_LIST, VertexIdType> {
public:
  GraphFile(const std::string&);

  const std::vector<std::pair<VertexIdType, VertexIdType>>&
  edgeList() const;

  const std::unordered_set<VertexIdType>&
  idSet() const;

private:
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;
  std::unordered_set<VertexIdType> m_idSet;
}; // class GraphFile<GraphFileType::EDGE_LIST, VertexIdType>

/**
 * @brief  Partial specialization of GraphFile class for FileType = GraphFileType::INCIDENCE_MATRIX.
 */
template <typename VertexIdType>
class GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType> {
public:
  GraphFile(const std::string&);

  const std::vector<std::pair<VertexIdType, VertexIdType>>&
  edgeList() const;

  const std::unordered_set<VertexIdType>&
  idSet() const;

private:
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;
  std::unordered_set<VertexIdType> m_idSet;
}; // class GraphFile<GraphFileType::INCIDENCE_MATRIX, VertexIdType>

/**
 * @brief  Partial specialization of GraphFile class for FileType = GraphFileType::ARG_DATABASE.
 */
template <typename VertexIdType>
class GraphFile<GraphFileType::ARG_DATABASE, VertexIdType> {
public:
  GraphFile(const std::string&);

  const std::vector<std::pair<VertexIdType, VertexIdType>>&
  edgeList() const;

  const std::unordered_set<VertexIdType>&
  idSet() const;

private:
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;
  std::unordered_set<VertexIdType> m_idSet;
}; // class GraphFile<GraphFileType::ARG_DATABASE, VertexIdType>

#endif // GRAPHFILE_HPP_
