/**
 * \file task_abort_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/ta/logical_task.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/events/robot_free_block_drop.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/tasks/task_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class task_abort_interactor
 * \ingroup support
 *
 * \brief Handles a robot's (possible) aborting of its current task on a given
 * timestep.
 */
template <typename T>
class task_abort_interactor : public rer::client<task_abort_interactor<T>> {
 public:
  task_abort_interactor(carena::caching_arena_map* const map,
                        tv::env_dynamics* envd,
                        argos::CFloorEntity* const floor)
      : ER_CLIENT_INIT("fordyca.support.task_abort_interactor"),
        m_map(map),
        m_envd(envd),
        m_floor(floor) {}

  /**
   * \brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * \todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  task_abort_interactor(const task_abort_interactor& other) = default;
  task_abort_interactor& operator=(const task_abort_interactor&) = delete;

  /**
   * \brief Handle cases in which a robot aborts its current task, and perform
   * any necessary cleanup, such as dropping/distributing a carried block, etc.
   *
   * \param controller The robot to handle task abort for.
   * \param penalty_handlers List of all penalty handlers in the arena that the
   *                         robot may be serving a penalty with that may need
   *                         to be updated.
   *
   * \return \c TRUE if the robot aborted is current task, \c FALSE otherwise.
   */
  bool operator()(T& controller) {
    if (nullptr == controller.current_task() ||
        tasks::task_status::ekABORT_PENDING != controller.task_status()) {
      return false;
    }
    RCSW_UNUSED auto polled =
        dynamic_cast<const cta::polled_task*>(controller.current_task());
    /*
     * If a robot aborted its task and was carrying a block, it needs to (1)
     * drop it so that the block is not left dangling and unusable for the rest
     * of the simulation, (2) update its own internal state.
     */
    if (controller.is_carrying_block()) {
      ER_INFO("%s aborted task '%s' while carrying block%d",
              controller.GetId().c_str(),
              polled->name().c_str(),
              controller.block()->id().v());
      task_abort_with_block(controller);
    } else {
      ER_INFO("%s aborted task '%s' (no block)",
              controller.GetId().c_str(),
              polled->name().c_str());
    }

    m_envd->penalties_flush(controller);
    return true;
  }

 private:
  void task_abort_with_block(T& controller) {
    auto loc =
        rmath::dvec2zvec(controller.rpos2D(), m_map->grid_resolution().v());
    rtypes::type_uuid block_id = controller.block()->id();
    events::robot_free_block_drop_visitor rdrop_op(controller.block_release(),
                                                   loc,
                                                   m_map->grid_resolution());

    caops::free_block_drop_visitor<crepr::base_block2D> adrop_op(
        m_map->blocks()[block_id.v()],
        loc,
        m_map->grid_resolution(),
        carena::arena_map_locking::ekNONE_HELD);

    adrop_op.visit(*m_map);
    rdrop_op.visit(controller);

    m_floor->SetChanged();
  } /* perform_block_drop() */

  /* clang-format off */
  carena::caching_arena_map* const m_map;
  tv::env_dynamics* const          m_envd;
  argos::CFloorEntity* const       m_floor;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_ */
