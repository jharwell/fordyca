/**
 * \file foraging_controller.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <any>
#include <memory>
#include <string>
#include <typeindex>

#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/controller/irv_recipient_controller.hpp"
#include "cosm/controller/manip_event_recorder.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"
#include "cosm/hal/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "cosm/pal/config/output_config.hpp"
#include "cosm/pal/controller/controller2D.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::config {
struct actuation_subsystem2D_config;
} // namespace cosm::subsystem::config
namespace cosm::steer2D::config {
struct force_calculator_config;
}
namespace cosm::tv {
class robot_dynamics_applicator;
}
namespace cosm::repr {
class unicell_entity2D;
} // namespace cosm::repr

namespace cosm::spatial {
class nest_zone_tracker;
} /* namespace cosm::spatial */

namespace rcppsw::math::config {
struct rng_config;
} // namespace rcppsw::math::config

namespace fordyca::repr {
class forager_los;
} /* namespace fordyca::repr */

NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_controller
 * \ingroup controller
 *
 * \brief The base controller foraging class that all FORDYCA controllers derive
 * from. It holds all functionality common to all controllers, as well that some
 * that is stubbed out here, but overridden in derived classes which allows this
 * class to be used as the robot controller handle when rendering QT graphics
 * overlays.
 */
class foraging_controller
    : public cpcontroller::controller2D,
      public ccontroller::block_carrying_controller,
      public cfsm::block_transporter<fsm::foraging_transport_goal>,
      public cfsm::metrics::block_transporter_metrics,
      public ccontroller::irv_recipient_controller,
      public rer::client<foraging_controller>,
      public csmetrics::nest_zone_metrics {
 public:
  using block_manip_recorder_type = ccontroller::manip_event_recorder<
      metrics::blocks::block_manip_events::ekMAX_EVENTS>;

  foraging_controller(void) RCPPSW_COLD;
  ~foraging_controller(void) override RCPPSW_COLD;

  foraging_controller(const foraging_controller&) = delete;
  foraging_controller& operator=(const foraging_controller&) = delete;

  /* controller2D overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void reset(void) override RCPPSW_COLD;

  /* rda_recipient_controller overrides */
  double applied_movement_throttle(void) const override final;
  void irv_init(const ctv::robot_dynamics_applicator* rda) override final;

  /* block carrying controller overrides */
  bool block_detect(const ccontroller::block_detect_context& context) override;

  /* movement metrics */
  rspatial::euclidean_dist
  ts_distance(const csmetrics::movement_category& category) const override;
  rmath::vector3d
  ts_velocity(const csmetrics::movement_category& category) const override;

  /* nest zone metrics */
  RCPPSW_WRAP_DECL_OVERRIDE(bool, in_nest, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, entered_nest, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, exited_nest, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::timestep, nest_duration, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::timestep, nest_entry_time, const);

  const block_manip_recorder_type* block_manip_recorder(void) const {
    return &m_block_manip;
  }
  block_manip_recorder_type* block_manip_recorder(void) { return &m_block_manip; }
  const cspatial::nest_zone_tracker* nz_tracker(void) const {
    return m_nz_tracker.get();
  }

 protected:
  cspatial::nest_zone_tracker* nz_tracker(void) { return m_nz_tracker.get(); }
  void block_detect_status_update(void);

 private:
  void
  saa_init(const csubsystem::config::actuation_subsystem2D_config* actuation,
           const chal::subsystem::config::sensing_subsystemQ3D_config* sensing);
  fs::path output_init(const cpconfig::output_config* outputp) override;

  /* clang-format off */
  bool                                         m_block_detect_status{false};
  block_manip_recorder_type                    m_block_manip{};
  std::unique_ptr<cspatial::nest_zone_tracker> m_nz_tracker{nullptr};
  /* clang-format on */
};

NS_END(controller, fordyca);
