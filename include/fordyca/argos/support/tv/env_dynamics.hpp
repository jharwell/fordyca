/**
 * \file env_dynamics.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "rcppsw/er/client.hpp"

#include "cosm/tv/temporal_penalty_handler.hpp"
#include "cosm/argos/tv/rda_adaptor.hpp"
#include "cosm/tv/env_dynamics.hpp"

#include "fordyca/argos/support/tv/block_op_penalty_handler.hpp"
#include "fordyca/argos/support/tv/cache_op_penalty_handler.hpp"
#include "fordyca/argos/support/tv/block_op_src.hpp"
#include "fordyca/argos/support/tv/cache_op_src.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics.hpp"
#include "fordyca//controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support);

namespace tv::config {
struct env_dynamics_config;
}
class argos_swarm_manager;

NS_START(tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics
 * \ingroup argos support tv
 *
 * \brief Orchestrates all application of temporal variance in environmental
 * conditions to the swarm.
 */
class env_dynamics final : public rer::client<env_dynamics>,
                           public ctv::env_dynamics<cpcontroller::controller2D>,
                           public fmetrics::tv::env_dynamics_metrics {
 public:
  using const_penalty_handlers = std::list<const ctv::temporal_penalty_handler*>;
  using penalty_handlers = std::list<ctv::temporal_penalty_handler*>;

  /**
   * \brief We use the \ref controller::foraging_controller, rather than the 2D
   * adaptor it is derived from because we need the \ref
   * ccontroller::irv_recipient_controller that it derives from within the RDA
   * class.
   */
  using rda_adaptor_type = cargos::tv::rda_adaptor<controller::foraging_controller>;

  env_dynamics(const fastv::config::env_dynamics_config * config,
               const fasupport::argos_swarm_manager* lf,
               carena::caching_arena_map* map);

  env_dynamics(const env_dynamics&) = delete;
  const env_dynamics& operator=(const env_dynamics&) = delete;

  /* environmental variance metrics */
  double avg_motion_throttle(void) const override {
    return m_rda.avg_motion_throttle();
  }
  rtypes::timestep arena_block_manip_penalty(void) const override;
  rtypes::timestep cache_usage_penalty(void) const override;

  /* COSM env dynamics overrides */
  void register_controller(const cpcontroller::controller2D& c) override;
  void unregister_controller(const cpcontroller::controller2D& c) override;
  bool penalties_flush(const cpcontroller::controller2D& c) override;

  /**
   * \brief Update the state of applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t) {
    m_timestep = t;
    m_rda.update();
  }


  const rda_adaptor_type* rda_adaptor(void) const { return &m_rda; }

  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of block operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  RCPPSW_PURE const block_op_penalty_handler* penalty_handler(
      const block_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler(src);
  }
  RCPPSW_PURE block_op_penalty_handler* penalty_handler(const block_op_src& src) {
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        return &m_fb_pickup;
      case block_op_src::ekNEST_DROP:
        return &m_nest_drop;
      case block_op_src::ekNEW_CACHE_DROP:
        return &m_new_cache;
      case block_op_src::ekCACHE_SITE_DROP:
        return &m_cache_site;
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", static_cast<int>(src));
    } /* switch() */
    return nullptr;
  }

  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of cache operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  RCPPSW_PURE const cache_op_penalty_handler* penalty_handler(
      const cache_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler(src);
  }

  RCPPSW_PURE cache_op_penalty_handler* penalty_handler(const cache_op_src& src) {
    switch (src) {
      case cache_op_src::ekEXISTING_CACHE_PICKUP:
      case cache_op_src::ekEXISTING_CACHE_DROP:
        return &m_existing_cache;
        break;
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", static_cast<int>(src));
    } /* switch() */
    return nullptr;
  }

  /**
   * \brief Get the full list of all possible penalty handlers. Note that for
   * some controller types there are handlers in the returned list for which a
   * controller can *NEVER* be serving a penalty for.
   */
  const_penalty_handlers all_penalty_handlers(void) const {
    return {penalty_handler(block_op_src::ekFREE_PICKUP),
          penalty_handler(block_op_src::ekNEST_DROP),
          penalty_handler(cache_op_src::ekEXISTING_CACHE_DROP),
          penalty_handler(block_op_src::ekCACHE_SITE_DROP),
          penalty_handler(block_op_src::ekNEW_CACHE_DROP),
          };
  }


 private:
  penalty_handlers all_penalty_handlers(void) {
    return {penalty_handler(block_op_src::ekFREE_PICKUP),
          penalty_handler(block_op_src::ekNEST_DROP),
          penalty_handler(cache_op_src::ekEXISTING_CACHE_DROP),
          penalty_handler(block_op_src::ekCACHE_SITE_DROP),
          penalty_handler(block_op_src::ekNEW_CACHE_DROP),
          };
  }

  /* clang-format off */
  rtypes::timestep         m_timestep{rtypes::timestep(0)};
  rda_adaptor_type         m_rda;
  block_op_penalty_handler m_fb_pickup;
  block_op_penalty_handler m_nest_drop;
  cache_op_penalty_handler m_existing_cache;
  block_op_penalty_handler m_new_cache;
  block_op_penalty_handler m_cache_site;
  /* clang-format on */
};

NS_END(tv, support, argos, fordyca);

