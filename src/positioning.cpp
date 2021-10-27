#include "autoware_v2x/positioning.hpp"
#include <vanetza/common/stored_position_provider.hpp>

using namespace vanetza;
namespace po = boost::program_options;

std::unique_ptr<vanetza::PositionProvider> create_position_provider(boost::asio::io_service &io_service, const Runtime &runtime)
{
  std::unique_ptr<vanetza::PositionProvider> positioning;

  std::unique_ptr<StoredPositionProvider> stored{new StoredPositionProvider()};
  PositionFix fix;
  fix.timestamp = runtime.now();
  fix.latitude = 10.0 * units::degree;
  fix.longitude = 10.0 * units::degree;
  fix.confidence.semi_major = 1.0 * units::si::meter;
  fix.confidence.semi_minor = fix.confidence.semi_major;
  stored->position_fix(fix);
  positioning = std::move(stored);

  return positioning;
}