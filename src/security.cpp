#include "autoware_v2x/security.hpp"

using namespace vanetza;

std::unique_ptr<security::SecurityEntity>
create_security_entity(const Runtime &runtime, PositionProvider &positioning)
{
  // Return nullptr - no security for now
  // TODO: Implement proper security with new Vanetza v3 API
  (void)runtime;
  (void)positioning;
  
  return nullptr;
}
