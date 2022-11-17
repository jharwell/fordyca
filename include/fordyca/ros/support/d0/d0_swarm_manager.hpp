/**
 * \file d0_swarm_manager.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/hal/robot.hpp"

#include "fordyca/ros/support/swarm_manager.hpp"
#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

namespace fordyca::ros::metrics::d0 {
class d0_swarm_metrics_manager;
} /* namespace fordyca::ros::metrics::d0 */

NS_START(fordyca, ros, support, d0);

namespace detail {
struct functor_maps_initializer;
} /* namespace detail */

template<typename Controller>
class robot_arena_interactor;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d0_swarm_manager
 * \ingroup ros support d0
 *
 * \brief Contains the support functions for d0 foraging, such
 * as:
 *
 * - Metric collection from robots
 */
class d0_swarm_manager : public rer::client<d0_swarm_manager>,
                         public frsupport::swarm_manager {
 public:
  d0_swarm_manager(const cros::config::sierra_config* config) RCPPSW_COLD;
  ~d0_swarm_manager(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void post_step(void) override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

 protected:
  /**
   * \brief Initialize d0 support to be shared with derived classes:
   *
   * - Depth0 metric collection
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

 private:
   /**
   * \brief Initialize d0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection (reflection
   *   basically).
   */
  void private_init(void) RCPPSW_COLD;

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Collect metrics from it.
   */
  void robot_post_step(chal::robot& robot);

  /* clang-format off */
  std::unique_ptr<frmetrics::d0::d0_swarm_metrics_manager> m_metrics_manager;
  /* clang-format on */
};

NS_END(d0, support, ros, fordyca);
