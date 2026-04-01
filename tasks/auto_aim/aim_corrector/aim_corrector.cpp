#include "aim_corrector.hpp"
#include "distance_corrector.hpp"
#include "accumulative_corrector.hpp"
#include "bullet_tracking_corrector.hpp"

namespace auto_aim
{

std::shared_ptr<AimCorrector> AimCorrectorFactory::create(
  Type type,
  const std::string & config_path)
{
  switch (type) {
    case Type::DISTANCE:
      return std::make_shared<DistanceCorrector>(config_path);
    case Type::ACCUMULATIVE:
      return std::make_shared<AccumulativeCorrector>(config_path);
    case Type::BULLET_TRACKING:
      return std::make_shared<BulletTrackingCorrector>(config_path);
    case Type::NONE:
    default:
      return nullptr;
  }
}

AimCorrectorFactory::Type AimCorrectorFactory::type_from_string(const std::string & str)
{
  if (str == "distance") return Type::DISTANCE;
  if (str == "accumulative") return Type::ACCUMULATIVE;
  if (str == "bullet_tracking") return Type::BULLET_TRACKING;
  return Type::NONE;
}

}  // namespace auto_aim