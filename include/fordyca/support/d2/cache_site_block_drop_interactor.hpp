/**
 * \file cache_site_block_drop_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/math/vector2.hpp"

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/controller/cognitive/d2/events/free_block_drop.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/support/d2/dynamic_cache_manager.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/support/cache_prox_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cache_site_block_drop_interactor
 * \ingroup support d2
 *
 * \brief Handles a robot's (possible) \ref free_block_drop event at a cache
 * site on a given timestep.
 */
template <typename TController, typename TControllerSpecMap>
class cache_site_block_drop_interactor : public rer::client<
  cache_site_block_drop_interactor<TController, TControllerSpecMap>
  > {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_free_block_drop_visitor_type =
      typename controller_spec::robot_free_block_drop_visitor_type;

  cache_site_block_drop_interactor(carena::caching_arena_map* const map_in,
                                   argos::CFloorEntity* const floor_in,
                                   tv::env_dynamics* envd,
                                   dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.d2.cache_site_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_cache_manager(cache_manager),
        m_penalty_handler(envd->penalty_handler(
            tv::block_op_src::ekCACHE_SITE_DROP)),
        m_prox_checker(map_in, m_cache_manager->cache_proximity_dist()) {}

  cache_site_block_drop_interactor(
      cache_site_block_drop_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  cache_site_block_drop_interactor(
      const cache_site_block_drop_interactor&) = delete;
  cache_site_block_drop_interactor& operator=(
      const cache_site_block_drop_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status operator()(TController& controller, const rtypes::timestep& t) {
    /*
     * If the controller was serving a penalty and has not finished yet, nothing
     * to do. If the controller was serving a penalty AND has satisfied it as of
     * this timestep, then actually perform the drop.
     */
    interactor_status status = interactor_status::ekNO_EVENT;
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        ER_ASSERT(pre_process_check(controller), "Pre-drop check failed");
        status = process_cache_site_block_drop(controller);
        ER_ASSERT(post_process_check(controller), "Post-drop check failed");
      }
    } else {
      auto prox_dist = boost::make_optional(m_cache_manager->cache_proximity_dist());
      auto penalty_status = m_penalty_handler->penalty_init(controller,
                                                    t,
                                                    tv::block_op_src::ekCACHE_SITE_DROP,
                                                    prox_dist);

      /*
       * The checking is redundant here, because it was already done during
       * penalty_init(), but it doesn't hurt. What we DO need from
       * check_and_notify() is the "notify" part. Without it, robots which try
       * to start caches too close to existing caches NEVER get the signal that
       * they are too close, and just sit waiting until they abort.
       *
       * See FORDYCA#684.
       */
      if (tv::op_filter_status::ekCACHE_PROXIMITY == penalty_status) {
        m_prox_checker.check_and_notify(controller, "cache_site");
      }
    }
    return status;
  }

 private:
  /**
   * \brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  interactor_status process_cache_site_block_drop(TController& controller) {
    const auto& penalty = m_penalty_handler->penalty_next();
    interactor_status status;

    if (m_prox_checker.check_and_notify(controller, "cache site")) {
      status = interactor_status::ekNO_EVENT;
    } else {
      execute_cache_site_block_drop(controller, penalty);
      status = interactor_status::ekFREE_BLOCK_DROP;
    }
    /*
     * Epilogue--need to remove the robot from the penalty list unconditionally,
     * because if it aborted the drop due to cache proximity, it will still be
     * on the list otherwise. See FORDYCA#669.
     */
    m_penalty_handler->penalty_remove(penalty);
    return status;
  }

  /**
   * \brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void execute_cache_site_block_drop(TController& controller,
                                     const ctv::temporal_penalty& penalty) {
    auto loc = rmath::dvec2zvec(controller.rpos2D(),
                                m_map->grid_resolution().v());
    /*
     * Safe to directly index into arena map block vector without locking
     * because the blocks never move from their original locations.
     */
    caops::free_block_drop_visitor adrop_op(m_map->blocks()[penalty.id().v()],
                                            loc,
                                            m_map->grid_resolution(),
                                            carena::locking::ekNONE_HELD);
    robot_free_block_drop_visitor_type rdrop_op(controller.block_release(),
                                                loc,
                                                m_map->grid_resolution());

    controller.block_manip_recorder()->record(metrics::blocks::block_manip_events::ekFREE_DROP,
                                              penalty.penalty());

    adrop_op.visit(*m_map);
    rdrop_op.visit(controller);

    m_floor->SetChanged();
  }

  bool pre_process_check(const TController& controller) const {
    const auto& penalty = m_penalty_handler->penalty_next();
    auto acq_goal = controller.current_task()->acquisition_goal();
    const auto * task = dynamic_cast<const events::dynamic_cache_interactor*>(
        controller.current_task());
    RCPPSW_UNUSED const auto * polled = dynamic_cast<const cta::polled_task*>(
        controller.current_task());

    ER_CHECK(penalty.controller() == &controller,
             "Out of order cache penalty handling");
    ER_CHECK(nullptr != task,
             "Non-cache interface task '%s'!",
             polled->name().c_str());
    ER_CHECK(controller.current_task()->goal_acquired() &&
             fsm::foraging_acq_goal::ekCACHE_SITE == acq_goal,
             "Robot%d@%s/%s not waiting for cache site block drop: acq_goal=%d",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str(),
             acq_goal.v());
    return true;

 error:
    return false;
  }

  bool post_process_check(const TController& controller) const {
    ER_CHECK(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving cache site penalty");

    return true;

 error:
    return false;
  } /* post_process_check() */

  /* clang-format off */
  argos::CFloorEntity*  const        m_floor;
  carena::caching_arena_map* const   m_map;
  dynamic_cache_manager*const        m_cache_manager;
  tv::block_op_penalty_handler*const m_penalty_handler;
  cache_prox_checker                 m_prox_checker;
  /* clang-format on */
};

NS_END(d2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_ */
