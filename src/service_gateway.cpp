#include "canopen_hw/service_gateway.hpp"

namespace canopen_hw {

namespace {

bool EnsureGatewayReady(OperationalCoordinator* coordinator, std::mutex* loop_mtx,
                        std_srvs::Trigger::Response* res) {
  if (res == nullptr) {
    return false;
  }
  if (!coordinator || !loop_mtx) {
    res->success = false;
    res->message = "service gateway not initialized";
    return false;
  }
  return true;
}

}  // namespace

ServiceGateway::ServiceGateway(ros::NodeHandle* pnh,
                               OperationalCoordinator* coordinator,
                               std::mutex* loop_mtx)
    : coordinator_(coordinator), loop_mtx_(loop_mtx) {
  if (pnh == nullptr) {
    return;
  }

  init_srv_ = pnh->advertiseService("init", &ServiceGateway::OnInit, this);
  halt_srv_ = pnh->advertiseService("halt", &ServiceGateway::OnHalt, this);
  resume_srv_ =
      pnh->advertiseService("resume", &ServiceGateway::OnResume, this);
  recover_srv_ =
      pnh->advertiseService("recover", &ServiceGateway::OnRecover, this);
  shutdown_srv_ =
      pnh->advertiseService("shutdown", &ServiceGateway::OnShutdown, this);
}

bool ServiceGateway::OnInit(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestInit();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0) ? "already initialized"
                                                         : "initialized";
  } else {
    res.message = r.message.empty() ? "init failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnHalt(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestHalt();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0) ? "already halted"
                                                         : "halted";
  } else {
    res.message = r.message.empty() ? "halt failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnResume(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestRelease();
  res.success = r.ok;
  if (r.ok) {
    res.message = "resumed";
  } else {
    res.message = r.message.empty() ? "resume failed; call ~/recover first"
                                    : r.message;
  }
  return true;
}

bool ServiceGateway::OnRecover(std_srvs::Trigger::Request&,
                               std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestRecover();
  res.success = r.ok;
  if (r.ok) {
    res.message = r.message.empty() ? "recovered" : r.message;
  } else {
    res.message = r.message.empty() ? "recover failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnShutdown(std_srvs::Trigger::Request&,
                                std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestShutdown();
  res.success = r.ok;
  if (r.ok) {
    res.message = "communication stopped; call ~/init first";
  } else {
    res.message = r.message.empty() ? "shutdown failed" : r.message;
  }
  return true;
}

}  // namespace canopen_hw
