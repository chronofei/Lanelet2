#include "lanelet2_traffic_rules/GenericTrafficRules.h"
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/utility/Units.h>
#include "lanelet2_traffic_rules/Exceptions.h"

namespace lanelet {
namespace traffic_rules {

namespace {
bool canChangeToLeft(LaneChangeType t) { return t == LaneChangeType::Both || t == LaneChangeType::ToLeft; }
bool canChangeToRight(LaneChangeType t) { return t == LaneChangeType::Both || t == LaneChangeType::ToRight; }

template <typename Map, typename Key, typename Value>
Value getMapOrDefault(const Map& map, Key key, Value defaultVal) {
  auto elem = map.find(key);
  if (elem == map.end()) {
    return defaultVal;
  }
  return elem->second;
}

// 从str的第一个字符开始比较，str是否包含substr
bool startswith(const std::string& str, const std::string& substr) {
  return str.compare(0, substr.size(), substr) == 0;
}

// 根据道路type、subtype及参与者判断当前道路是否支持换道
LaneChangeType getChangeType(const std::string& type, const std::string& subtype, const std::string& participant) {
  using LaneChangeMap = std::map<std::pair<std::string, std::string>, LaneChangeType>;
  const static LaneChangeMap VehicleChangeType{
      {{AttributeValueString::LineThin, AttributeValueString::Dashed}, LaneChangeType::Both},
      {{AttributeValueString::LineThick, AttributeValueString::Dashed}, LaneChangeType::Both},
      {{AttributeValueString::LineThin, AttributeValueString::DashedSolid}, LaneChangeType::ToRight},
      {{AttributeValueString::LineThick, AttributeValueString::DashedSolid}, LaneChangeType::ToRight},
      {{AttributeValueString::LineThin, AttributeValueString::SolidDashed}, LaneChangeType::ToLeft},
      {{AttributeValueString::LineThick, AttributeValueString::SolidDashed}, LaneChangeType::ToLeft}};
  const static LaneChangeMap PedestrianChangeType{
      {{AttributeValueString::Curbstone, AttributeValueString::Low}, LaneChangeType::Both}};

  if (startswith(participant, Participants::Vehicle)) {
    return getMapOrDefault(VehicleChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  if (participant == Participants::Pedestrian) {
    return getMapOrDefault(PedestrianChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  if (participant == Participants::Bicycle) {
    auto asVehicle = getMapOrDefault(VehicleChangeType, std::make_pair(type, subtype), LaneChangeType::None);
    if (asVehicle != LaneChangeType::None) {
      return asVehicle;
    }
    return getMapOrDefault(PedestrianChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  return LaneChangeType::None;
}

// 从lanelet的边界属性上判断是否可以改变车道
Optional<LaneChangeType> getHardcodedChangeType(const ConstLineString3d& boundary) {
  if (boundary.hasAttribute(AttributeNamesString::LaneChange)) {
    if (boundary.attributeOr(AttributeNamesString::LaneChange, false)) {
      return {true, LaneChangeType::Both};
    }
    return {true, LaneChangeType::None};
  }
  if (boundary.hasAttribute(AttributeNamesString::LaneChangeLeft)) {
    if (boundary.attributeOr(AttributeNamesString::LaneChangeLeft, false)) {
      if (boundary.attributeOr(AttributeNamesString::LaneChangeRight, false)) {
        return {true, LaneChangeType::Both};
      }
      return {true, LaneChangeType::ToLeft};
    }
    return {true, LaneChangeType::None};
  }
  if (boundary.hasAttribute(AttributeNamesString::LaneChangeRight)) {
    if (boundary.attributeOr(AttributeNamesString::LaneChangeRight, false)) {
      return {true, LaneChangeType::ToRight};
    }
    return {true, LaneChangeType::None};
  }
  return {false, LaneChangeType::None};
}

// attrs任何一个属性名字是否包含overridePrefix
bool hasOverride(const AttributeMap& attrs, const std::string& overridePrefix) {
  return utils::anyOf(attrs, [&overridePrefix](auto& attr) { return startswith(attr.first, overridePrefix); });
}

template <typename T>
T getOverride(const AttributeMap& attrs, const std::string& overridePrefix, const std::string& override, T defaultVal) {
  auto overrideAttr = utils::findIf(attrs, [&override, &overridePrefix](auto& attr) {
    // it is forbidden to define overrides at different hierachical levels, so we just have to search for one single
    // hit.
    if (attr.first.size() < overridePrefix.size()) {
      return false;
    }
    return startswith(override, attr.first);
  });
  if (!overrideAttr) {
    return defaultVal;
  }
  return overrideAttr->second.template as<T>().get_value_or(defaultVal);
}

// 判断是否可以沿着lanelet的方向行使,只是一个方向的判断
// 可以看成是构造图结构的时候，lanelet所代表的这条有向边是否存在
bool isDrivingDir(const lanelet::ConstLanelet& ll, const std::string& participant) {
  if (!ll.inverted()) {
    return true;
  }
  auto hasOneWay = ll.attributeOr(AttributeName::OneWay, Optional<bool>());
  if (!!hasOneWay) {
    return !*hasOneWay;
  }
  // 判断ll的属性中有没有定义OneWay
  if (hasOverride(ll.attributes(), AttributeNamesString::OneWay)) {
    return !getOverride(ll.attributes(), AttributeNamesString::OneWay,
                        AttributeNamesString::OneWay + (":" + participant), true);
  }
  // Participants结构在lanelet2_core模块的Attribute.h文件夹中定义
  return participant == Participants::Pedestrian;
}

template <typename T>
T sort(const T& toSort) {
  auto sorted = toSort;
  std::sort(sorted.begin(), sorted.end());
  return sorted;
}

}  // namespace
TrafficRules::~TrafficRules() = default;

bool GenericTrafficRules::hasDynamicRules(const ConstLanelet& lanelet) const {
  auto regelems = lanelet.regulatoryElements();
  auto isDynamic = [](const auto& elem) { return elem->attributeOr(AttributeName::Dynamic, false); };
  return std::any_of(regelems.begin(), regelems.end(), isDynamic);
}

// 首先是驾驶方向的判断，接着时交通规则的判断，在时车道类型与交通参与者的判断，最后是交通类型和地区的判断
bool GenericTrafficRules::canPass(const lanelet::ConstLanelet& lanelet) const {
  if (!isDrivingDir(lanelet, participant())) {
    return false;
  }
  auto canPassByRule = canPass(lanelet.regulatoryElements());
  if (!!canPassByRule) {
    return *canPassByRule;
  }
  if (hasOverride(lanelet.attributes(), AttributeNamesString::Participant)) {
    return getOverride(lanelet.attributes(), AttributeNamesString::Participant, Participants::tag(participant()),
                       false);
  }
  return canPass(lanelet.attributeOr(AttributeName::Subtype, ""), lanelet.attributeOr(AttributeName::Location, ""))
      .get_value_or(false);
}

// 与lanelet相比，area无方向，所以直接从交通规则开始判断，然后是交通参与者，最后才是通过交通类型和所在地区进行判断
bool GenericTrafficRules::canPass(const ConstArea& area) const {
  auto canPassByRule = canPass(area.regulatoryElements());
  if (!!canPassByRule) {
    return *canPassByRule;
  }
  if (hasOverride(area.attributes(), AttributeNamesString::Participant)) {
    return getOverride(area.attributes(), AttributeNamesString::Participant, Participants::tag(participant()), false);
  }
  return canPass(area.attributeOr(AttributeName::Subtype, ""), area.attributeOr(AttributeName::Location, ""))
      .get_value_or(false);
}

// 首先根据边界属性是否定义了可变道，若没有定义则通过边界线的type和subtype进行推断是否可以边界
LaneChangeType GenericTrafficRules::laneChangeType(const ConstLineString3d& boundary,
                                                   bool virtualIsPassable = false) const {
  using namespace std::string_literals;
  LaneChangeType changeType;
  auto result = getHardcodedChangeType(boundary);
  if (!!result) {
    changeType = *result;
  } else {
    auto type = boundary.attributeOr(AttributeName::Type, ""s);
    if (virtualIsPassable && type == AttributeValueString::Virtual) {
      return LaneChangeType::Both;
    }
    // 根据边界的线性和交通参与者是否判断是否可以变道
    changeType = getChangeType(type, boundary.attributeOr(AttributeName::Subtype, ""s), participant());
  }
  // handle inverted ls
  if (boundary.inverted()) {
    if (changeType == LaneChangeType::ToLeft) {
      return LaneChangeType::ToRight;
    }
    if (changeType == LaneChangeType::ToRight) {
      return LaneChangeType::ToLeft;
    }
  }
  return changeType;
}

// geometry::follows判断from和to是否是前驱和后继的关系
bool GenericTrafficRules::canPass(const ConstLanelet& from, const ConstLanelet& to) const {
  return geometry::follows(from, to) && canPass(from) && canPass(to);
}

Optional<ConstLineString3d> determineCommonLine(const ConstLanelet& ll, const ConstArea& ar) {
  return utils::findIf(ar.outerBound(), [p1 = ll.leftBound().back(), p2 = ll.rightBound().back()](auto& boundLs) {
    return (boundLs.back() == p1 && boundLs.front() == p2);
  });
}
Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar1, const ConstArea& ar2) {
  return utils::findIf(ar1.outerBound(), [&ar2](auto& ar1Bound) {
    return !!utils::findIf(ar2.outerBound(),
                           [ar1Bound = ar1Bound.invert()](auto& ar2Bound) { return ar2Bound == ar1Bound; });
  });
}

bool GenericTrafficRules::canPass(const ConstLanelet& from, const ConstArea& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  if (geometry::leftOf(from, to)) {
    return canChangeToLeft(laneChangeType(from.leftBound(), true));
  }
  if (geometry::rightOf(from, to)) {
    return canChangeToRight(laneChangeType(from.rightBound(), true));
  }
  auto line = determineCommonLine(from, to);
  if (!!line) {
    return canChangeToRight(laneChangeType(*line, true));
  }
  return false;
}

bool GenericTrafficRules::canPass(const ConstArea& from, const ConstLanelet& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  if (geometry::leftOf(to, from)) {
    return canChangeToRight(laneChangeType(to.leftBound(), true));
  }
  if (geometry::rightOf(to, from)) {
    return canChangeToLeft(laneChangeType(to.rightBound(), true));
  }
  auto line = determineCommonLine(to.invert(), from);
  if (!!line) {
    return canChangeToLeft(laneChangeType(*line, true));
  }
  return false;
}

// ????
bool GenericTrafficRules::canPass(const ConstArea& from, const ConstArea& to) const {
  if (!canPass(from) && canPass(to)) {
    return false;
  }
  auto line = determineCommonLine(from, to);
  if (!line) {
    return false;
  }
  return canChangeToLeft(laneChangeType(*line, true));
}

bool GenericTrafficRules::canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  bool isLeft = false;
  if (geometry::leftOf(from, to)) {
    isLeft = true;
  } else if (!geometry::rightOf(from, to)) {
    return false;
  }
  auto type = laneChangeType(isLeft ? from.rightBound() : from.leftBound());
  return isLeft ? canChangeToRight(type) : canChangeToLeft(type);
}

// 根据交通参与者和交通所在位置，确定道路速度限制
SpeedLimitInformation getSpeedLimitFromType(const AttributeMap& attributes, const CountrySpeedLimits& countryLimits,
                                            const std::string& participant) {
  using Value = AttributeValueString;
  using SpeedLimitMap = std::map<std::pair<std::string, std::string>, SpeedLimitInformation(CountrySpeedLimits::*)>;
  const static SpeedLimitMap SpeedLimitLookup{
      {{Value::Urban, Value::Road}, &CountrySpeedLimits::vehicleUrbanRoad},
      {{Value::Nonurban, Value::Road}, &CountrySpeedLimits::vehicleNonurbanRoad},
      {{Value::Urban, Value::Highway}, &CountrySpeedLimits::vehicleUrbanHighway},
      {{Value::Nonurban, Value::Highway}, &CountrySpeedLimits::vehicleNonurbanHighway},
      {{Value::Urban, Value::PlayStreet}, &CountrySpeedLimits::playStreet},
      {{Value::Nonurban, Value::PlayStreet}, &CountrySpeedLimits::playStreet},
      {{Value::Urban, Value::Exit}, &CountrySpeedLimits::vehicleUrbanRoad},
  };
  if (participant == Participants::Pedestrian) {
    return countryLimits.pedestrian;
  }
  if (participant == Participants::Bicycle) {
    return countryLimits.bicycle;
  }
  const std::string vehicle = Participants::Vehicle;
  if (startswith(participant, vehicle)) {
    auto location =
        getMapOrDefault(attributes, AttributeName::Location, Attribute(AttributeValueString::Urban)).value();
    auto type = getMapOrDefault(attributes, AttributeName::Subtype, Attribute(AttributeValueString::Road)).value();
    auto limit = SpeedLimitLookup.find(std::make_pair(location, type));
    if (limit != SpeedLimitLookup.end()) {
      return countryLimits.*(limit->second);
    }
  }
  return {};
}

// 首先根据交通规则，其次参考道路属性，最后，根据交通参与者与道路所在位置，判断道路速度限制
SpeedLimitInformation GenericTrafficRules::speedLimit(const RegulatoryElementConstPtrs& regelems,
                                                      const AttributeMap& attributes) const {
  using namespace std::string_literals;
  using namespace units::literals;
  using Attr = AttributeNamesString;
  auto regelemSpeedLimit = speedLimit(regelems);  // 此处返回交通规则中定义的速度限制
  if (!!regelemSpeedLimit) {
    return *regelemSpeedLimit;
  }
  if (hasOverride(attributes, Attr::SpeedLimit) || hasOverride(attributes, Attr::SpeedLimitMandatory)) {
    auto defaultLimit =
        getMapOrDefault(attributes, AttributeName::SpeedLimit, Attribute(0_kmh)).asVelocity().get_value_or(0_kmh);
    auto limit =
        getOverride(attributes, Attr::SpeedLimit + ":"s, Attr::SpeedLimit + ":"s + participant(), defaultLimit);
    auto mandatory =
        getOverride(attributes, Attr::SpeedLimitMandatory, Attr::SpeedLimitMandatory + ":"s + participant(), true);
    return {limit, mandatory};
  }
  return getSpeedLimitFromType(attributes, countrySpeedLimits(), participant());
}

SpeedLimitInformation GenericTrafficRules::speedLimit(const ConstLanelet& lanelet) const {
  return speedLimit(lanelet.regulatoryElements(), lanelet.attributes());
}

SpeedLimitInformation GenericTrafficRules::speedLimit(const ConstArea& area) const {
  return speedLimit(area.regulatoryElements(), area.attributes());
}

const std::string& TrafficRules::participant() const { return config_.at("participant").value(); }

const std::string& TrafficRules::location() const { return config_.at("location").value(); }

// 此处的类型为道路类型，首先定义了道路类型及其参与者，然后根据type确定潜在道路参与者，最后根据当前的道路参与者判断是否可以在此道路同行
Optional<bool> GenericTrafficRules::canPass(const std::string& type, const std::string& /*location*/) const {
  using ParticantsMap = std::map<std::string, std::vector<std::string>>;
  using Value = AttributeValueString;
  const static ParticantsMap ParticipantMap{
      {"", {Participants::Vehicle}},
      {Value::Road, {Participants::Vehicle, Participants::Bicycle}},
      {Value::Highway, {Participants::Vehicle}},
      {Value::BicycleLane, {Participants::Bicycle}},
      {Value::PlayStreet, {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {Value::EmergencyLane, {Participants::VehicleEmergency}},
      {Value::Exit, {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {Value::Walkway, {Participants::Pedestrian}},
      {Value::Crosswalk, {Participants::Pedestrian}},
      {Value::Stairs, {Participants::Pedestrian}},
      {Value::SharedWalkway, {Participants::Pedestrian, Participants::Bicycle}}};
  auto participants = ParticipantMap.find(type);
  if (participants == ParticipantMap.end()) {
    return {};
  }
  return utils::anyOf(participants->second,
                      [this](auto& participant) { return startswith(this->participant(), participant); });
}

bool GenericTrafficRules::isOneWay(const ConstLanelet& lanelet) const {
  return isDrivingDir(lanelet, participant()) != isDrivingDir(lanelet.invert(), participant());
}

std::ostream& operator<<(std::ostream& stream, const SpeedLimitInformation& obj) {
  return stream << "speedLimit: " << units::KmHQuantity(obj.speedLimit).value()
                << "km/h, mandatory: " << (obj.isMandatory ? "yes" : "no");
}

std::ostream& operator<<(std::ostream& stream, const TrafficRules& obj) {
  return stream << "location: " << obj.location() << ", participant: " << obj.participant();
}

}  // namespace traffic_rules
}  // namespace lanelet
