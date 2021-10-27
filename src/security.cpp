#include "autoware_v2x/security.hpp"
#include <vanetza/security/certificate_cache.hpp>
#include <vanetza/security/default_certificate_validator.hpp>
#include <vanetza/security/delegating_security_entity.hpp>
#include <vanetza/security/naive_certificate_provider.hpp>
#include <vanetza/security/null_certificate_validator.hpp>
#include <vanetza/security/persistence.hpp>
#include <vanetza/security/sign_header_policy.hpp>
#include <vanetza/security/static_certificate_provider.hpp>
#include <vanetza/security/trust_store.hpp>
#include <stdexcept>

using namespace vanetza;
namespace po = boost::program_options;

class SecurityContext : public security::SecurityEntity
{
public:
  SecurityContext(const Runtime &runtime, PositionProvider &positioning) : runtime(runtime), positioning(positioning),
                                                                           backend(security::create_backend("default")),
                                                                           sign_header_policy(runtime, positioning),
                                                                           cert_cache(runtime),
                                                                           cert_validator(*backend, cert_cache, trust_store)
  {
  }

  security::EncapConfirm encapsulate_packet(security::EncapRequest &&request) override
  {
    if (!entity)
    {
      throw std::runtime_error("security entity is not ready");
    }
    return entity->encapsulate_packet(std::move(request));
  }

  security::DecapConfirm decapsulate_packet(security::DecapRequest &&request) override
  {
    if (!entity)
    {
      throw std::runtime_error("security entity is not ready");
    }
    return entity->decapsulate_packet(std::move(request));
  }

  void build_entity()
  {
    if (!cert_provider)
    {
      throw std::runtime_error("certificate provider is missing");
    }
    security::SignService sign_service = straight_sign_service(*cert_provider, *backend, sign_header_policy);
    security::VerifyService verify_service = straight_verify_service(runtime, *cert_provider, cert_validator,
                                                                     *backend, cert_cache, sign_header_policy, positioning);
    entity.reset(new security::DelegatingSecurityEntity{sign_service, verify_service});
  }

  const Runtime &runtime;
  PositionProvider &positioning;
  std::unique_ptr<security::Backend> backend;
  std::unique_ptr<security::SecurityEntity> entity;
  std::unique_ptr<security::CertificateProvider> cert_provider;
  security::DefaultSignHeaderPolicy sign_header_policy;
  security::TrustStore trust_store;
  security::CertificateCache cert_cache;
  security::DefaultCertificateValidator cert_validator;
};

std::unique_ptr<security::SecurityEntity>
create_security_entity(const Runtime &runtime, PositionProvider &positioning)
{
  std::unique_ptr<security::SecurityEntity> security;

  return security;
}
