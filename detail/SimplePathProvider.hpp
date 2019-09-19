/**
 * @file SimplePathProvider.hpp
 * @brief Details of simple path provider functions.
 */
#ifndef DETAIL_SIMPLEPATHPROVIDER_HPP_
#define DETAIL_SIMPLEPATHPROVIDER_HPP_


/**
 * @brief  Partial specialization of SimplePathProvider class for Boost graphs.
 */
template <template <typename, typename> class GraphType, typename VertexProperties, typename VertexIdType>
class SimplePathProvider<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>> {
private:
  using Vertex = typename ::Vertex<GraphType, VertexProperties, VertexIdType>;
  using OutEdgeIterator = typename ::Edge<GraphType, VertexProperties, VertexIdType>::OutEdgeIterator;

public:
  /**
   * @brief  Constructs the provider for paths starting from a path.
   *
   * @param s  Source vertex for all the simple paths.
   * @param l  Length of all the simple paths.
   */
  SimplePathProvider(
    const Vertex& s,
    const VertexIdType& l
  ) : m_stack(),
      m_visited(1, s.id()),
      m_pathLength(l)
  {
    if (m_pathLength > 0) {
      auto outEdgeIt = s.outEdges();
      m_stack.push_back(std::make_pair(outEdgeIt.begin(), outEdgeIt.end()));
    }
  }

  /**
   * @brief Returns the next simple path after pruning.
   *
   * @param pruner  Function which returns true if a path is to be pruned.
   */
  template <typename PrunerType>
  std::vector<VertexIdType>
  next(
    const PrunerType& pruner
  )
  {
    while (!m_stack.empty()) {
      std::pair<OutEdgeIterator, OutEdgeIterator>& children = *(m_stack.rbegin());
      if (children.first == children.second) {
        m_visited.pop_back();
        m_stack.pop_back();
      }
      else {
        Vertex child = (*(children.first++)).target();
        if (m_visited.size() < m_pathLength) {
          if (std::find(m_visited.begin(), m_visited.end(), child.id()) == m_visited.end()) {
            m_visited.push_back(child.id());
            auto outEdgeIt = child.outEdges();
            m_stack.push_back(std::make_pair(outEdgeIt.begin(), outEdgeIt.end()));
          }
        }
        else {
          std::vector<VertexIdType> path(m_visited);
          m_visited.pop_back();
          m_stack.pop_back();
          if (pruner && pruner(path)) {
            continue;
          }
          else {
            return path;
          }
        }
      }
    }
    return std::vector<VertexIdType>();
  }

  /**
   * @brief Returns the next simple path without any pruning.
   */
  std::vector<VertexIdType>
  next(
  )
  {
    return this->next([] (const std::vector<VertexIdType>&) { return false; });
  }

private:
  std::vector<std::pair<OutEdgeIterator, OutEdgeIterator>> m_stack;
  std::vector<VertexIdType> m_visited;
  const VertexIdType m_pathLength;
}; // class SimplePathProvider<GraphType, VertexProperties, VertexIdType, EnableBoostAll<GraphType, VertexProperties, VertexIdType>>

#endif // DETAIL_SIMPLEPATHPROVIDER_HPP_
