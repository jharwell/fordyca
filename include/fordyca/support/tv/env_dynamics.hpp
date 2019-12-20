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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_ENV_DYNAMICS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_ENV_DYNAMICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/support/tv/block_op_penalty_handler.hpp"
#include "fordyca/support/tv/cache_op_penalty_handler.hpp"
#include "fordyca/support/tv/block_op_src.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics.hpp"
#include "fordyca/support/tv/argos_rda_adaptor.hpp"

#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace config { namespace tv { struct env_dynamics_config; }}
namespace ds { class arena_map; }
NS_START(support);

class base_loop_functions;

NS_START(tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics
 * \ingroup support tv
 *
 * \brief Orchestrates all application of temporal variance in environmental
 * conditions to the swarm.
 */
class env_dynamics final : public rer::client<env_dynamics>,
                           public metrics::tv::env_dynamics_metrics {
 public:
  using const_penalty_handlers = std::list<const tv::temporal_penalty_handler*>;
  using penalty_handlers = std::list<tv::temporal_penalty_handler*>;

  env_dynamics(const config::tv::env_dynamics_config * config,
               const support::base_loop_functions* lf,
               ds::arena_map* map);

  env_dynamics(const env_dynamics&) = delete;
  const env_dynamics& operator=(const env_dynamics&) = delete;

  /* environmental variance metrics */
  double avg_motion_throttle(void) const override {
    return m_rda.avg_motion_throttle();
  }
  rtypes::timestep block_manip_penalty(void) const override;
  rtypes::timestep cache_usage_penalty(void) const override;

  /**
   * \brief Update the state of applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t) {
    m_timestep = t;
    m_rda.update();
  }

  /**
   * \brief Register a robot controller for all possible types of environmental
   * variance that could be applied to it.
   */
  void register_controller(const controller::base_controller& c);

  /**
   * \brief Undo \ref register_controller, as well as flushing the controller
   * from any penalty handlers it might be serving penalties for.
   */
  void unregister_controller(const controller::base_controller& c);

  bool penalties_flush(const controller::base_controller& c);

  const argos_rda_adaptor* rda_adaptor(void) const { return &m_rda; }

  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of block operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  const block_op_penalty_handler* penalty_handler(const block_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler(src);
  }
   block_op_penalty_handler* penalty_handler(const block_op_src& src) {
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
  const cache_op_penalty_handler* penalty_handler(const cache_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler(src);
  }

  cache_op_penalty_handler* penalty_handler(const cache_op_src& src) {
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
  argos_rda_adaptor        m_rda;
  block_op_penalty_handler m_fb_pickup;
  block_op_penalty_handler m_nest_drop;
  cache_op_penalty_handler m_existing_cache;
  block_op_penalty_handler m_new_cache;
  block_op_penalty_handler m_cache_site;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_ENV_DYNAMICS_HPP_ */
