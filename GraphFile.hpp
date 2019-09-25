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
 * @brief Enumeration class for different graph file types.
 */
enum class GraphFileType {
  EDGE_LIST,
  INCIDENCE_MATRIX,
  ARG_DATABASE,
  FHCP
};

/**
 * @brief Class that provides the functionality for reading a graph file.
 *
 * @tparam FileType      The type of file to be read.
 * @tparam VertexIdType  Unsigned type for storing vertex ids.
 */
template <enum GraphFileType FileType, typename VertexIdType>
class GraphFile;

#define DECL_GRAPH_FILE(FileType)\
template <typename VertexIdType>\
class GraphFile<FileType, VertexIdType> {\
public:\
  GraphFile(const std::string&);\
\
  const std::vector<std::pair<VertexIdType, VertexIdType>>&\
  edgeList() const;\
\
  const std::unordered_set<VertexIdType>&\
  idSet() const;\
\
private:\
  std::vector<std::pair<VertexIdType, VertexIdType>> m_edgeList;\
  std::unordered_set<VertexIdType> m_idSet;\
}

/**
 * @brief Partial specialization of GraphFile class for FileType = GraphFileType::EDGE_LIST.
 */
DECL_GRAPH_FILE(GraphFileType::EDGE_LIST);

/**
 * @brief Partial specialization of GraphFile class for FileType = GraphFileType::INCIDENCE_MATRIX.
 */
DECL_GRAPH_FILE(GraphFileType::INCIDENCE_MATRIX);

/**
 * @brief Partial specialization of GraphFile class for FileType = GraphFileType::ARG_DATABASE.
 */
DECL_GRAPH_FILE(GraphFileType::ARG_DATABASE);

/**
 * @brief Partial specialization of GraphFile class for FileType = GraphFileType::FHCP.
 */
DECL_GRAPH_FILE(GraphFileType::FHCP);

#endif // GRAPHFILE_HPP_
