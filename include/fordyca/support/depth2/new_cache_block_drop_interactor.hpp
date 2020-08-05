/**
 * \file new_cache_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/ta/polled_task.hpp"
#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/events/robot_free_block_drop.hpp"
#include "fordyca/support/cache_prox_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class new_cache_block_drop_interactor
 * \ingroup support depth2
 *
 * \brief Handles a robot's (possible) \ref free_block_drop event at a new cache
 * on a given timestep.
 */
template <typename TController>
class new_cache_block_drop_interactor : public rer::client<new_cache_block_drop_interactor<TController>> {
 public:
  new_cache_block_drop_interactor(carena::caching_arena_map* const map_in,
                                  argos::CFloorEntity* const floor_in,
                                  tv::env_dynamics* const envd,
                                  dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.new_cache_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_cache_manager(cache_manager),
        m_penalty_handler(envd->penalty_handler(
            tv::block_op_src::ekNEW_CACHE_DROP)),
        m_prox_checker(map_in, m_cache_manager->cache_proximity_dist()) {}

  new_cache_block_drop_interactor(
      new_cache_block_drop_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  new_cache_block_drop_interactor(
      const new_cache_block_drop_interactor&) = default;
  new_cache_block_drop_interactor& operator=(
      const new_cache_block_drop_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t   The current timestep.
   *
   * \return \c TRUE if a block was dropped in a new cache, \c FALSE otherwise.
   */
  interactor_status operator()(TController& controller, const rtypes::timestep& t) {
    interactor_status status = interactor_status::ekNO_EVENT;
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        ER_ASSERT(pre_process_check(controller), "Pre-drop check failed");
        status = process_new_cache_block_drop(controller);
        ER_ASSERT(post_process_check(controller), "Post-drop check failed");
      }
    } else {
      auto prox_dist = boost::make_optional(m_cache_manager->cache_proximity_dist());
      m_penalty_handler->penalty_init(controller,
                                      t,
                                      tv::block_op_src::ekNEW_CACHE_DROP,
                                      prox_dist);
    }
    return status;
  }

 private:
  /**
   * \brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  interactor_status process_new_cache_block_drop(TController& controller) {
    const auto& penalty = m_penalty_handler->penalty_next();
    interactor_status status;

    if (m_prox_checker.check_and_notify(controller, "new cache")) {
      status = interactor_status::ekNO_EVENT;
    } else {
      execute_new_cache_block_drop(controller, penalty);
      status = interactor_status::ekNEW_CACHE_BLOCK_DROP;
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
  void execute_new_cache_block_drop(TController& controller,
                                    const ctv::temporal_penalty& penalty) {
    auto loc = rmath::dvec2zvec(controller.rpos2D(),
                                m_map->grid_resolution().v());

    caops::free_block_drop_visitor adrop_op(m_map->blocks()[penalty.id().v()],
                                            loc,
                                            m_map->grid_resolution(),
                                            carena::arena_map_locking::ekNONE_HELD);
    events::robot_free_block_drop_visitor rdrop_op(controller.block_release(),
                                                   loc,
                                                   m_map->grid_resolution());

    controller.block_manip_recorder()->record(metrics::blocks::block_manip_events::ekFREE_DROP,
                                              penalty.penalty());

    rdrop_op.visit(controller);
    adrop_op.visit(*m_map);
    m_floor->SetChanged();
  }

  bool pre_process_check(const TController& controller) const {
    const auto& penalty = m_penalty_handler->penalty_next();
    auto acq_goal = controller.current_task()->acquisition_goal();
    auto* task = dynamic_cast<const events::dynamic_cache_interactor*>(
        controller.current_task());
    RCSW_UNUSED auto* polled = dynamic_cast<const cta::polled_task*>(
        controller.current_task());

    ER_CHECK(penalty.controller() == &controller,
             "Out of order cache penalty handling");
    ER_CHECK(nullptr != task,
             "Non-cache interface task '%s'!",
             polled->name().c_str());
    ER_CHECK(controller.current_task()->goal_acquired() &&
             fsm::foraging_acq_goal::ekNEW_CACHE == acq_goal,
             "Robot%d@%s/%s not waiting for new cache block drop: acq_goal=%d",
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
              "Multiple instances of same controller serving new cache drop penalty");

    return true;

 error:
    return false;
  }

  /* clang-format off */
  argos::CFloorEntity*  const         m_floor;
  carena::caching_arena_map* const    m_map;
  dynamic_cache_manager*const         m_cache_manager;
  tv::block_op_penalty_handler* const m_penalty_handler;
  cache_prox_checker                  m_prox_checker;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_ */
