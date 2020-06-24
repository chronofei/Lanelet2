#include "lanelet2_routing/RoutingGraph.h"
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <algorithm>
#include <boost/graph/reverse_graph.hpp>
#include <cassert>  // Asserts
#include <memory>
#include <queue>
#include <utility>
#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/Forward.h"
#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/internal/Graph.h"
#include "lanelet2_routing/internal/GraphUtils.h"
#include "lanelet2_routing/internal/RouteBuilder.h"
#include "lanelet2_routing/internal/RoutingGraphBuilder.h"
#include "lanelet2_routing/internal/ShortestPath.h"

// needs to be included after shotestPath due to some overload resolution quirks
#include "lanelet2_routing/internal/RoutingGraphVisualization.h"

namespace lanelet {
namespace routing {

#if __cplusplus < 201703L
constexpr const char RoutingGraph::ParticipantHeight[];
#endif

namespace {
using internal::DijkstraSearchMap;
using internal::DijkstraStyleSearch;
using internal::FilteredRoutingGraph;
using internal::GraphType;
using internal::LaneletVertexId;
using internal::RoutingGraphGraph;
using internal::VertexVisitInformation;

// 调整容器的大小为size
template <typename T>
T reservedVector(size_t size) {
  T t;
  t.reserve(size);
  return t;
}

/** @brief Helper objecct to save state of possible routes search */
struct PossibleRoutesInfo;
using PossibleRoutesInfoUPtr = std::unique_ptr<PossibleRoutesInfo>;
struct PossibleRoutesInfo {
  uint32_t vertexId{};
  double totalDistance{0};
  size_t numLaneChanges{0};
  ConstLaneletOrAreas laneletsOrAreas;
  explicit PossibleRoutesInfo(uint32_t startVertex, const ConstLaneletOrArea& startLanelet) : vertexId(startVertex) {
    laneletsOrAreas.emplace_back(startLanelet);
  }
};

//! sort functor for possible routes
struct MoreLaneChanges {
  bool operator()(const PossibleRoutesInfo& lhs, const PossibleRoutesInfo& rhs) const {
    if (lhs.numLaneChanges == rhs.numLaneChanges) {
      return lhs.totalDistance > rhs.totalDistance;
    }
    return lhs.numLaneChanges > rhs.numLaneChanges;
  }
};

/** @brief Simple helper function to combine shortest paths */
template <typename T, typename U>
bool addToPath(T& path, const Optional<U>& newElements) {
  if (newElements) {
    path.insert(path.end(), ++newElements->begin(), newElements->end());
    return true;  // NOLINT
  }
  return false;
}

//! Helper function to create a new point that represents a lanelet.
template <typename PointT>
PointT createPoint(const ConstLaneletOrArea& ll) {
  PointT p;
  p.setId(ll.id());
  p.setAttribute("id", Attribute(ll.id()));
  if (ll.isLanelet()) {
    boost::geometry::centroid(utils::toHybrid(ll.lanelet()->polygon2d()), p);
  }
  if (ll.isArea()) {
    boost::geometry::centroid(utils::toHybrid(utils::to2D(ll.area()->outerBoundPolygon())), p);
  }
  return p;
}

// 在图graph中找到顶点vertex的出度边，找到第一个出度边所对应的汇点的laneletOrArea
/** @brief Implementation function to retrieve a neighboring vertex
 *  @throws RoutingGraphError if 'throwOnError' is true and there is more than one neighboring lanelet
 *  @param vertex Start vertex
 *  @param graph Filtered graph with the allowed type of edge
 *  @param throwOnError Decides wheter to throw or ignore an error
 *  @return Neighboring lanelet */
Optional<ConstLaneletOrArea> neighboringImpl(const GraphType::vertex_descriptor vertex,
                                             const FilteredRoutingGraph& graph, bool throwOnError = false) {
  auto outEdges = boost::out_edges(vertex, graph);
  if (throwOnError && std::distance(outEdges.first, outEdges.second) > 1) {
    std::string ids;
    std::for_each(outEdges.first, outEdges.second, [&graph, &ids](const auto& edge) {
      ids += " " + std::to_string(graph[boost::target(edge, graph)].laneletOrArea.id());
    });
    throw RoutingGraphError("More than one neighboring lanelet to " + std::to_string(graph[vertex].laneletOrArea.id()) +
                            " with this relation:" + ids);
  }
  if (outEdges.first != outEdges.second) {
    return graph[boost::target(*(outEdges.first), graph)].laneletOrArea;
  }
  return {};
}

// 在图中找到一个邻居顶点，并且这个邻居顶点是lanelet
Optional<ConstLanelet> neighboringLaneletImpl(const GraphType::vertex_descriptor vertex,
                                              const FilteredRoutingGraph& graph, bool throwOnError = false) {
  auto value = neighboringImpl(vertex, graph, throwOnError);
  if (!!value && value->isLanelet()) {
    return value->lanelet();
  }
  return {};
}

// 判断llt是否在图g中，若在，则执行函数f
template <typename Func>
Optional<ConstLaneletOrArea> ifInGraph(const RoutingGraphGraph& g, const ConstLaneletOrArea& llt, Func f) {
  auto vertex = g.getVertex(llt);
  if (!vertex) {
    return {};
  }
  return f(*vertex);
}

// 判断llt是否在图g中，若在则执行函数f
template <typename Func>
Optional<ConstLanelet> ifLaneletInGraph(const RoutingGraphGraph& g, const ConstLanelet& llt, Func f) {
  auto laneletVertex = g.getVertex(llt);
  if (!laneletVertex) {
    return {};
  }
  return f(*laneletVertex);
}

// 一直执行函数next，直到函数返回类型未初始化
template <typename Func>
ConstLanelets getUntilEnd(const ConstLanelet& start, Func next) {
  auto result = reservedVector<ConstLanelets>(3);
  Optional<ConstLanelet> current = start;
  while (!!(current = next(*current))) {
    result.emplace_back(*current);
  }
  return result;
}

// 在图graph中找到与laneletOrArea相邻的laneletOrArea，根据edgesOut，判断是出度边的邻居，还是入度边的邻居
ConstLaneletOrAreas getAllEdgesFromGraph(const RoutingGraphGraph& graph, const FilteredRoutingGraph& subgraph,
                                         const ConstLaneletOrArea& laneletOrArea, bool edgesOut) {
  ConstLaneletOrAreas result;
  auto laneletVertex = graph.getVertex(laneletOrArea);
  if (!laneletVertex) {
    return result;
  }
  auto processEdges = [&](auto edgeRange) {
    result.reserve(size_t(std::distance(edgeRange.first, edgeRange.second)));
    for (; edgeRange.first != edgeRange.second; edgeRange.first++) {
      auto node =
          edgesOut ? boost::target(*edgeRange.first, graph.get()) : boost::source(*edgeRange.first, graph.get());
      result.emplace_back(graph.get()[node].laneletOrArea);
    }
    return result;
  };
  return edgesOut ? processEdges(boost::out_edges(*laneletVertex, subgraph))
                  : processEdges(boost::in_edges(*laneletVertex, subgraph));
}

// 在图graph中找到lanelet的邻居，根据edgeOut，判断是出度边的邻居，还是入度边的邻居，且邻居必须是lanelet
ConstLanelets getLaneletEdgesFromGraph(const RoutingGraphGraph& graph, const FilteredRoutingGraph& subgraph,
                                       const ConstLanelet& lanelet, bool edgesOut) {
  ConstLanelets result;
  auto allEdges = getAllEdgesFromGraph(graph, subgraph, lanelet, edgesOut);
  result = reservedVector<ConstLanelets>(allEdges.size());
  for (auto& edge : allEdges) {
    if (edge.isLanelet()) {
      result.push_back(*edge.lanelet());
    }
  }
  return result;
}

template <bool Backw>
struct GetGraph {};
template <>
struct GetGraph<true> {
  template <typename G>
  auto operator()(const G& g) {
    return boost::make_reverse_graph(g);
  }
};
template <>
struct GetGraph<false> {
  template <typename G>
  auto operator()(const G& g) {
    return g;
  }
};

// 根据dijkstra生成的最短路径树，最终生成最短路径
template <bool Backw, typename OutVertexT, typename GraphT>
std::vector<OutVertexT> buildPath(const DijkstraSearchMap<LaneletVertexId>& map, LaneletVertexId vertex, GraphT g) {
  const auto* currInfo = &map.at(vertex);
  auto size = currInfo->length;
  std::vector<OutVertexT> path(size);
  while (true) {
    auto idx = Backw ? size - currInfo->length : currInfo->length - 1;
    path[idx] = static_cast<OutVertexT>(g[vertex].laneletOrArea);
    if (currInfo->predecessor == vertex) {
      break;
    }
    vertex = currInfo->predecessor;
    currInfo = &map.at(vertex);
  }
  return path;
}

// 从start出发计算最短路径，有些点是无法到达的，返回到这些点的可能路径
template <bool Backw, typename OutVertexT, typename OutContainerT, typename Func>
std::vector<OutContainerT> possiblePathsImpl(const GraphType::vertex_descriptor& start,
                                             const FilteredRoutingGraph& graph, Func stopCriterion) {
  auto g = GetGraph<Backw>{}(graph);
  DijkstraStyleSearch<decltype(g)> search(g);
  search.query(start, stopCriterion);
  auto keepPath = [&](auto& vertex) { return vertex.second.isLeaf && !vertex.second.predicate; };
  auto numPaths = size_t(std::count_if(search.getMap().begin(), search.getMap().end(), keepPath));
  std::vector<OutContainerT> result;
  result.reserve(numPaths);
  for (auto& vertex : search.getMap()) {
    if (!keepPath(vertex)) {
      continue;
    }
    result.emplace_back(buildPath<Backw, OutVertexT>(search.getMap(), vertex.first, graph));
  }
  return result;
}

// 从start出发，在图中找到能到达的点，返回点的集合
template <bool Backw, typename OutVertexT, typename Func>
std::vector<OutVertexT> reachableSetImpl(const GraphType::vertex_descriptor& start, const FilteredRoutingGraph& graph,
                                         Func stopCriterion) {
  auto g = GetGraph<Backw>{}(graph);
  DijkstraStyleSearch<decltype(g)> search(g);
  search.query(start, stopCriterion);
  std::vector<OutVertexT> result;
  result.reserve(search.getMap().size());
  for (auto& vertex : search.getMap()) {
    if (vertex.second.predicate) {
      result.emplace_back(static_cast<OutVertexT>(graph[vertex.first].laneletOrArea));
    }
  }
  return result;
}

// 若lanelet的数量超过n则返回false
template <bool Eq = false>
struct StopIfLaneletsMoreThan {
  explicit StopIfLaneletsMoreThan(size_t n) : n{n} {}
  template <typename T>
  inline bool operator()(const T& v) const {
    return Eq ? v.length <= n : v.length < n;
  }
  size_t n;
};
// 若代v的代价值超过c则返回false
template <bool Eq = false>
struct StopIfCostMoreThan {
  explicit StopIfCostMoreThan(double c) : c{c} {}
  template <typename T>
  inline bool operator()(const T& v) const {
    return Eq ? v.cost <= c : v.cost < c;
  }
  double c;
};

// 基于from、to和dijkstra算法，实现最短路径
template <typename PathT, typename PrimT>
Optional<PathT> shortestPathImpl(const PrimT& from, const PrimT& to, RoutingCostId routingCostId, bool withLaneChanges,
                                 bool withAreas, const internal::RoutingGraphGraph& graph) {
  auto startVertex = graph.getVertex(from);
  auto endVertex = graph.getVertex(to);
  if (!startVertex || !endVertex) {
    return {};
  }
  auto filteredGraph =
      withLaneChanges
          ? withAreas ? graph.withAreasAndLaneChanges(routingCostId) : graph.withLaneChanges(routingCostId)
          : withAreas ? graph.withAreasWithoutLaneChanges(routingCostId) : graph.withoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(filteredGraph);
  class DestinationReached {};
  try {
    search.query(*startVertex, [endVertex](const internal::VertexVisitInformation& i) {
      if (i.vertex == *endVertex) {
        throw DestinationReached{};
      }
      return true;
    });
  } catch (DestinationReached) {  // NOLINT
    return PathT{buildPath<false, PrimT>(search.getMap(), *endVertex, filteredGraph)};
  }
  return {};
}

// 实现最短路径，但是要经过某一个lanelet
template <typename RetT, typename Primitives, typename ShortestPathFunc>
Optional<RetT> shortestPathViaImpl(Primitives routePoints, ShortestPathFunc&& shortestPath) {
  Primitives path;
  for (size_t it = 0; it < routePoints.size() - 1; it++) {
    auto results = shortestPath(routePoints[it], routePoints[it + 1]);
    if (!!results && !results->empty() && path.empty()) {
      path.push_back(results->front());
    }
    if (!addToPath(path, results)) {
      return Optional<RetT>();
    }
  }
  return RetT(path);
}
}  // namespace

RoutingGraph::RoutingGraph(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph& RoutingGraph::operator=(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph::~RoutingGraph() = default;

// 根据laneletMap，trafficRules, routingCosts构建图结构
RoutingGraphUPtr RoutingGraph::build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                     const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config) {
  return internal::RoutingGraphBuilder(trafficRules, routingCosts, config).build(laneletMap);
}

// 根据laneletMap，trafficRules, routingCosts构建图结构
RoutingGraphUPtr RoutingGraph::build(const LaneletSubmap& laneletSubmap,
                                     const traffic_rules::TrafficRules& trafficRules,
                                     const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config) {
  return internal::RoutingGraphBuilder(trafficRules, routingCosts, config).build(laneletSubmap);
}

// 根据最短路径，实现路由
Optional<Route> RoutingGraph::getRoute(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId,
                                       bool withLaneChanges) const {
  auto optPath{shortestPath(from, to, routingCostId, withLaneChanges)};
  if (!optPath) {
    return {};
  }
  return internal::RouteBuilder(*graph_).getRouteFromShortestPath(*optPath, withLaneChanges, routingCostId);
}

// 通过最短路径实现路由，但是路由要经过某一个lanelet
Optional<Route> RoutingGraph::getRouteVia(const ConstLanelet& from, const ConstLanelets& via, const ConstLanelet& to,
                                          RoutingCostId routingCostId, bool withLaneChanges) const {
  auto optPath{shortestPathVia(from, via, to, routingCostId, withLaneChanges)};
  if (!optPath) {
    return {};
  }
  return internal::RouteBuilder(*graph_).getRouteFromShortestPath(*optPath, withLaneChanges, routingCostId);
}

// 获取从from到to的最短路径 LaneletPath
Optional<LaneletPath> RoutingGraph::shortestPath(const ConstLanelet& from, const ConstLanelet& to,
                                                 RoutingCostId routingCostId, bool withLaneChanges) const {
  return shortestPathImpl<LaneletPath, ConstLanelet>(from, to, routingCostId, withLaneChanges, false, *graph_);
}

// 实现最短路径，但是包含Area
Optional<LaneletOrAreaPath> RoutingGraph::shortestPathIncludingAreas(const ConstLaneletOrArea& from,
                                                                     const ConstLaneletOrArea& to,
                                                                     RoutingCostId routingCostId,
                                                                     bool withLaneChanges) const {
  return shortestPathImpl<LaneletOrAreaPath, ConstLaneletOrArea>(from, to, routingCostId, withLaneChanges, true,
                                                                 *graph_);
}

// 实现最短路径，但是最短路径中包含某一个lanelet
Optional<LaneletPath> RoutingGraph::shortestPathVia(const ConstLanelet& start, const ConstLanelets& via,
                                                    const ConstLanelet& end, RoutingCostId routingCostId,
                                                    bool withLaneChanges) const {
  ConstLanelets routePoints = utils::concatenate({ConstLanelets{start}, via, ConstLanelets{end}});
  return shortestPathViaImpl<LaneletPath>(
      routePoints, [&](auto& from, auto& to) { return this->shortestPath(from, to, routingCostId, withLaneChanges); });
}

// 实现最短路径，路径可以经过area，且路径必须通过某一个lanelet or area
Optional<LaneletOrAreaPath> RoutingGraph::shortestPathIncludingAreasVia(const ConstLaneletOrArea& start,
                                                                        const ConstLaneletOrAreas& via,
                                                                        const ConstLaneletOrArea& end,
                                                                        RoutingCostId routingCostId,
                                                                        bool withLaneChanges) const {
  ConstLaneletOrAreas routePoints = utils::concatenate({ConstLaneletOrAreas{start}, via, ConstLaneletOrAreas{end}});
  return shortestPathViaImpl<LaneletOrAreaPath>(routePoints, [&](auto& from, auto& to) {
    return this->shortestPathIncludingAreas(from, to, routingCostId, withLaneChanges);
  });
}

// 找到两个lanelet在图中所对应的边，然后返回边的关系
Optional<RelationType> RoutingGraph::routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                                     bool includeConflicting) const {
  auto edgeInfo = includeConflicting ? graph_->getEdgeInfo(from, to)
                                     : graph_->getEdgeInfoFor(from, to, graph_->withoutConflicting());
  if (!!edgeInfo) {
    return edgeInfo->relation;
  }
  return {};
}

// 在图中找到lanelet的出度边邻居
ConstLanelets RoutingGraph::following(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto subgraph = withLaneChanges ? graph_->withLaneChanges() : graph_->withoutLaneChanges();
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, true);
}

// 在图中找到lanelet的出度边邻居，返回当前lanelet与其出度边邻居的关系
LaneletRelations RoutingGraph::followingRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets foll{following(lanelet, withLaneChanges)};
  LaneletRelations result;
  for (auto const& it : foll) {
    result.emplace_back(LaneletRelation{it, *routingRelation(lanelet, it)});
  }
  return result;
}  // namespace routing

// 在图中找到lanelet的入度边邻居
ConstLanelets RoutingGraph::previous(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto subgraph = withLaneChanges ? graph_->withLaneChanges(0) : graph_->withoutLaneChanges(0);
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, false);
}

// 在图中找到lanelet的入度边邻居，返回当前lanelet与其入度边邻居的关系
LaneletRelations RoutingGraph::previousRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets prev{previous(lanelet, withLaneChanges)};
  LaneletRelations result;
  result.reserve(prev.size());
  for (auto const& it : prev) {
    Optional<RelationType> relation{routingRelation(it, lanelet)};
    if (!!relation) {
      result.emplace_back(LaneletRelation{it, *relation});
    } else {
      assert(false && "Two Lanelets in a route are not connected. This shouldn't happen.");  // NOLINT
    }
  }
  return result;
}

ConstLanelets RoutingGraph::besides(const ConstLanelet& lanelet) const {
  auto move = [](auto it) { return std::make_move_iterator(it); };
  ConstLanelets left{lefts(lanelet)};
  ConstLanelets right{rights(lanelet)};
  ConstLanelets result;
  result.reserve(left.size() + right.size() + 1);
  result.insert(std::end(result), move(left.rbegin()), move(left.rend()));
  result.push_back(lanelet);
  result.insert(std::end(result), move(std::begin(right)), move(std::end(right)));
  return result;
}

// 返回lanelet的一个左邻居
Optional<ConstLanelet> RoutingGraph::left(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [this](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->left()); });
}

// 返回lanelet的相邻的一个左邻居
Optional<ConstLanelet> RoutingGraph::adjacentLeft(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [this](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->adjacentLeft()); });
}

// 返回lanelet的一个右邻居
Optional<ConstLanelet> RoutingGraph::right(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [this](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->right()); });
}

// 返回lanelet的相邻的一个右邻居
Optional<ConstLanelet> RoutingGraph::adjacentRight(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [this](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->adjacentRight()); });
}

// 找到lanelet所对应的所有左邻居
ConstLanelets RoutingGraph::lefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return left(llt); });
}

// 找到lanelet所对应的所有相邻的左邻居
ConstLanelets RoutingGraph::adjacentLefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return adjacentLeft(llt); });
}

// 返回lanelet左边所有邻居的关系
LaneletRelations RoutingGraph::leftRelations(const ConstLanelet& lanelet) const {
  bool leftReached{false};
  ConstLanelet current = lanelet;
  LaneletRelations result;
  while (!leftReached) {
    const ConstLanelets leftOf{lefts(current)};
    for (auto const& it : leftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Left});
      current = it;
    }
    const ConstLanelets adjacentLeftOf{adjacentLefts(current)};
    for (auto const& it : adjacentLeftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentLeft});
      current = it;
    }
    leftReached = (leftOf.empty() && adjacentLeftOf.empty());
  }
  return result;
}

// 返回lanelet所对应的所有右邻居
ConstLanelets RoutingGraph::rights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return right(llt); });
}

// 返回lanelet所对应的所有相邻的右邻居
ConstLanelets RoutingGraph::adjacentRights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return adjacentRight(llt); });
}

// 返回lanelet所对应的所有右边邻居的关系
LaneletRelations RoutingGraph::rightRelations(const ConstLanelet& lanelet) const {
  bool rightReached{false};
  ConstLanelet current = lanelet;
  auto result = reservedVector<LaneletRelations>(3);
  while (!rightReached) {
    const ConstLanelets rightOf{rights(current)};
    for (auto const& it : rightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Right});
      current = it;
    }
    const ConstLanelets adjacentRightOf{adjacentRights(current)};
    for (auto const& it : adjacentRightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentRight});
      current = it;
    }
    rightReached = (rightOf.empty() && adjacentRightOf.empty());
  }
  return result;
}

// 返回所有与laneletOrArea的关系为conflicting的laneletOrArea
ConstLaneletOrAreas RoutingGraph::conflicting(const ConstLaneletOrArea& laneletOrArea) const {
  return getAllEdgesFromGraph(*graph_, graph_->conflicting(), laneletOrArea, true);
}

// 返回从lanelet出发，在图中能到达的lanelet的集合，且代价值不超过maxRoutingCost
ConstLanelets RoutingGraph::reachableSet(const ConstLanelet& lanelet, double maxRoutingCost,
                                         RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return reachableSetImpl<false, ConstLanelet>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

// 返回从lanelet出发，在图中能到达的lanelet的集合，且代价值不超过maxRoutingCost
ConstLaneletOrAreas RoutingGraph::reachableSetIncludingAreas(const ConstLaneletOrArea& llOrAr, double maxRoutingCost,
                                                             RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(llOrAr);
  if (!start) {
    return {};
  }
  auto graph = graph_->withAreasAndLaneChanges(routingCostId);
  return reachableSetImpl<false, ConstLaneletOrArea>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

// 返回能到达lanelet的lanelet的集合，且代价值不能超过maxRoutingCost
ConstLanelets RoutingGraph::reachableSetTowards(const ConstLanelet& lanelet, double maxRoutingCost,
                                                RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return reachableSetImpl<true, ConstLanelet>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

// 从startPoint出发，计算最短路径，但是由于限制条件有些点无法到达，该方法计算从startPoint到这些点的可能路径
LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, double minRoutingCost,
                                         RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return possiblePathsImpl<false, ConstLanelet, LaneletPath>(*start, graph, StopIfCostMoreThan<>{minRoutingCost});
}

// 同上，但是限制条件为lanelet的个数，而不是代价值
LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, uint32_t minLanelets, bool allowLaneChanges,
                                         RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return possiblePathsImpl<false, ConstLanelet, LaneletPath>(*start, graph, StopIfLaneletsMoreThan<>{minLanelets});
}

// 同上，只是所用的图，经过了旋转
LaneletPaths RoutingGraph::possiblePathsTowards(const ConstLanelet& targetLanelet, double minRoutingCost,
                                                RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(targetLanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return possiblePathsImpl<true, ConstLanelet, LaneletPath>(*start, graph, StopIfCostMoreThan<>{minRoutingCost});
}

// 同上，只是所用的代价为最小的lanelet数量
LaneletPaths RoutingGraph::possiblePathsTowards(const ConstLanelet& targetLanelet, uint32_t minLanelets,
                                                bool allowLaneChanges, RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(targetLanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return possiblePathsImpl<true, ConstLanelet, LaneletPath>(*start, graph, StopIfLaneletsMoreThan<>{minLanelets});
}

// 可能路径，但是经过Area, 限制条件为代价值
LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint,
                                                             double minRoutingCost, RoutingCostId routingCostId,
                                                             bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                : graph_->withAreasWithoutLaneChanges(routingCostId);
  return possiblePathsImpl<false, ConstLaneletOrArea, LaneletOrAreaPath>(*start, graph,
                                                                         StopIfCostMoreThan<>{minRoutingCost});
}

// 同上，限制条件为lanelet数量
LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, uint32_t minElements,
                                                             bool allowLaneChanges, RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                : graph_->withAreasWithoutLaneChanges(routingCostId);
  return possiblePathsImpl<false, ConstLaneletOrArea, LaneletOrAreaPath>(*start, graph,
                                                                         StopIfLaneletsMoreThan<>{minElements});
}

void RoutingGraph::forEachSuccessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges,
                                    RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet(), graph_->get()[i.predecessor].lanelet(), i.cost,
                                     i.length, i.numLaneChanges});
  });
}

void RoutingGraph::forEachSuccessorIncludingAreas(const ConstLaneletOrArea& lanelet,
                                                  const LaneletOrAreaVisitFunction& f, bool allowLaneChanges,
                                                  RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                : graph_->withAreasWithoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletOrAreaVisitInformation{graph_->get()[i.vertex].laneletOrArea,
                                           graph_->get()[i.predecessor].laneletOrArea, i.cost, i.length,
                                           i.numLaneChanges});
  });
}

void RoutingGraph::forEachPredecessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges,
                                      RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto forwGraph =
      allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  auto graph = boost::make_reverse_graph(forwGraph);  // forwGraph needs to stay on the stack
  internal::DijkstraStyleSearch<decltype(graph)> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet(), graph_->get()[i.predecessor].lanelet(), i.cost,
                                     i.length, i.numLaneChanges});
  });
}

void RoutingGraph::forEachPredecessorIncludingAreas(const ConstLaneletOrArea& lanelet,
                                                    const LaneletOrAreaVisitFunction& f, bool allowLaneChanges,
                                                    RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto forwGraph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                    : graph_->withAreasWithoutLaneChanges(routingCostId);
  auto graph = boost::make_reverse_graph(forwGraph);  // forwGraph needs to stay on the stack
  internal::DijkstraStyleSearch<decltype(graph)> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletOrAreaVisitInformation{graph_->get()[i.vertex].laneletOrArea,
                                           graph_->get()[i.predecessor].laneletOrArea, i.cost, i.length,
                                           i.numLaneChanges});
  });
}

void RoutingGraph::exportGraphML(const std::string& filename, const RelationType& edgeTypesToExclude,
                                 RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphMLImpl<GraphType>(filename, graph_->get(), relations, routingCostId);
}

void RoutingGraph::exportGraphViz(const std::string& filename, const RelationType& edgeTypesToExclude,
                                  RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphVizImpl<GraphType>(filename, graph_->get(), relations, routingCostId);
}

//! Helper function to slim down getDebugLaneletMap
RelationType allowedRelationsfromConfiguration(bool includeAdjacent, bool includeConflicting) {
  RelationType allowedRelations{RelationType::Successor | RelationType::Left | RelationType::Right |
                                RelationType::Area};
  if (includeAdjacent) {
    allowedRelations |= RelationType::AdjacentLeft;
    allowedRelations |= RelationType::AdjacentRight;
  }
  if (includeConflicting) {
    allowedRelations |= RelationType::Conflicting;
  }
  return allowedRelations;
}

LineString3d createLineString(const Point2d& from, const Point2d& to, RelationType relation, double routingCost) {
  LineString2d lineString(utils::getId());
  lineString.push_back(from);
  lineString.push_back(to);
  LineString3d lineString3d(lineString);
  lineString3d.setAttribute("relation", relationToString(relation));
  lineString3d.setAttribute("routing_cost", routingCost);
  return lineString3d;
}

class DebugMapBuilder {
 public:
  using LaneletOrAreaPair = std::pair<ConstLaneletOrArea, ConstLaneletOrArea>;
  explicit DebugMapBuilder(const FilteredRoutingGraph& graph) : graph_{graph} {}
  LaneletMapPtr run(const internal::LaneletOrAreaToVertex& loa) {
    LaneletMapPtr output = std::make_shared<LaneletMap>();
    for (const auto& vertex : loa) {
      visitVertex(vertex);
    }
    auto lineStrings = utils::transform(lineStringMap_, [](auto& mapLs) { return mapLs.second; });
    auto map = utils::createMap(lineStrings);
    for (auto& p : pointMap_) {
      map->add(utils::to3D(p.second));
    }
    return map;
  }

 private:
  void visitVertex(const internal::LaneletOrAreaToVertex::value_type& vertex) {
    addPoint(vertex.first);
    auto edges = boost::out_edges(vertex.second, graph_);
    for (auto edge = edges.first; edge != edges.second; ++edge) {
      const auto& target = graph_[boost::target(*edge, graph_)].laneletOrArea;
      addPoint(target);
      const auto& edgeInfo = graph_[*edge];
      addEdge(vertex.first, target, edgeInfo);
    }
  }

  static LaneletOrAreaPair getPair(const ConstLaneletOrArea& first, const ConstLaneletOrArea& second) {
    return first.id() < second.id() ? LaneletOrAreaPair(first, second) : LaneletOrAreaPair(second, first);
  }

  void addPoint(const ConstLaneletOrArea& point) {
    auto inMap = pointMap_.find(point);
    if (inMap == pointMap_.end()) {
      pointMap_.emplace(point, createPoint<Point2d>(point));
    }
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, internal::EdgeInfo edge) {
    auto pair = getPair(from, to);
    auto inMap = lineStringMap_.find(pair);
    if (inMap != lineStringMap_.end()) {
      inMap->second.setAttribute("relation_reverse", relationToString(edge.relation));
      inMap->second.setAttribute("routing_cost_reverse", std::to_string(edge.routingCost));

    } else {
      auto pFrom = pointMap_.at(from);
      auto pTo = pointMap_.at(to);
      LineString3d lineString3d{createLineString(pFrom, pTo, edge.relation, edge.routingCost)};
      lineStringMap_.emplace(pair, lineString3d);
    }
  }

  FilteredRoutingGraph graph_;
  std::unordered_map<LaneletOrAreaPair, LineString3d> lineStringMap_;  // Stores all relations
  std::unordered_map<ConstLaneletOrArea, Point2d> pointMap_;           // Stores all 'edges'
};

LaneletMapPtr RoutingGraph::getDebugLaneletMap(RoutingCostId routingCostId, bool includeAdjacent,
                                               bool includeConflicting) const {
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  internal::EdgeCostFilter<GraphType> edgeFilter(
      graph_->get(), routingCostId, allowedRelationsfromConfiguration(includeAdjacent, includeConflicting));
  FilteredRoutingGraph filteredGraph(graph_->get(), edgeFilter);
  return DebugMapBuilder(filteredGraph).run(graph_->vertexLookup());
}

RoutingGraph::Errors RoutingGraph::checkValidity(bool throwOnError) const {
  Errors errors;
  for (const auto& laWithVertex : graph_->vertexLookup()) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;
    auto id = la.id();
    // Check left relation
    Optional<ConstLanelet> left;
    try {
      left = neighboringLaneletImpl(vertex, graph_->left(), true);

    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Left: ") + e.what());
    }
    Optional<ConstLanelet> adjacentLeft;
    try {
      adjacentLeft = neighboringLaneletImpl(vertex, graph_->adjacentLeft(), true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Adjacent left: ") + e.what());
    }
    if (left && adjacentLeft) {
      errors.emplace_back("Lanelet " + std::to_string(id) + " has both 'left' (id: " + std::to_string(left->id()) +
                          ") and 'adjancent_left' (id: " + std::to_string(adjacentLeft->id()) + ") lanelet");
    }
    if (left) {
      LaneletRelations rel{rightRelations(*left)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'left' relation from " + std::to_string(id) + " to " +
                            std::to_string(left->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'left' relation from " + std::to_string(id) + " to " +
                            std::to_string(left->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    if (adjacentLeft) {
      LaneletRelations rel{rightRelations(*adjacentLeft)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'adjacentLeft' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentLeft->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'adjacentLeft' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentLeft->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    // Check right
    Optional<ConstLanelet> right;
    try {
      right = neighboringLaneletImpl(vertex, graph_->right(), true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Right: ") + e.what());
    }
    Optional<ConstLanelet> adjacentRight;
    try {
      adjacentRight = neighboringLaneletImpl(vertex, graph_->adjacentRight(), true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Adjacent right: ") + e.what());
    }
    if (right && adjacentRight) {
      errors.emplace_back("Lanelet " + std::to_string(id) + " has both 'right' (id: " + std::to_string(right->id()) +
                          ") and 'adjancent_right' (id: " + std::to_string(adjacentRight->id()) + ") lanelet");
    }
    if (right) {
      LaneletRelations rel{leftRelations(*right)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'right' relation from " + std::to_string(id) + " to " +
                            std::to_string(right->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'right' relation from " + std::to_string(id) + " to " +
                            std::to_string(right->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    if (adjacentRight) {
      LaneletRelations rel{leftRelations(*adjacentRight)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'adjacentRight' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentRight->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'adjacentRight' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentRight->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
  }
  if (throwOnError && !errors.empty()) {
    std::stringstream ss;
    ss << "Errors found in routing graph:";
    for (const auto& err : errors) {
      ss << "\n\t- " << err;
    }
    throw RoutingGraphError(ss.str());
  }
  return errors;
}

//RoutingGraph构造函数，仅仅初始化了两个私有变量
RoutingGraph::RoutingGraph(std::unique_ptr<RoutingGraphGraph>&& graph, LaneletSubmapConstPtr&& passableMap)
    : graph_{std::move(graph)}, passableLaneletSubmap_{std::move(passableMap)} {}

}  // namespace routing
}  // namespace lanelet
