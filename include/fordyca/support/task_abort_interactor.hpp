/**
 * @file task_abort_interactor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include <argos3/core/simulator/entity/floor_entity.h>
#include <list>
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/support/tv/tv_manager.hpp"
#include "fordyca/tasks/task_status.hpp"
#include "rcppsw/ta/logical_task.hpp"
#include "rcppsw/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class task_abort_interactor
 * @ingroup fordyca support
 *
 * @brief Handles a robot's (possible) aborting of its current task on a given
 * timestep.
 */
template <typename T>
class task_abort_interactor : public rer::client<task_abort_interactor<T>> {
 public:
  using penalty_handler_list = std::list<tv::temporal_penalty_handler<T>*>;

  task_abort_interactor(ds::arena_map* const map_in,
                        argos::CFloorEntity* const floor_in)
      : ER_CLIENT_INIT("fordyca.support.task_abort_interactor"),
        m_map(map_in),
        m_floor(floor_in) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  task_abort_interactor(const task_abort_interactor& other) = default;
  task_abort_interactor& operator=(const task_abort_interactor& other) = delete;

  /**
   * @brief Handle cases in which a robot aborts its current task, and perform
   * any necessary cleanup, such as dropping/distributing a carried block, etc.
   *
   * @param controller The robot to handle task abort for.
   * @param penalty_handlers List of all penalty handlers in the arena that the
   *                         robot may be serving a penalty with that may need
   *                         to be updated.
   *
   * @return \c TRUE if the robot aborted is current task, \c FALSE otherwise.
   */
  bool operator()(T& controller, const penalty_handler_list& penalty_handlers) {
    if (nullptr == controller.current_task() ||
        tasks::task_status::ekAbortPending != controller.task_status()) {
      return false;
    }
    RCSW_UNUSED auto polled =
        dynamic_cast<const rta::polled_task*>(controller.current_task());
    /*
     * If a robot aborted its task and was carrying a block, it needs to (1)
     * drop it so that the block is not left dangling and unusable for the rest
     * of the simulation, (2) update its own internal state.
     */
    if (controller.is_carrying_block()) {
      ER_INFO("%s aborted task '%s' while carrying block%d",
              controller.GetId().c_str(),
              polled->name().c_str(),
              controller.block()->id());
      task_abort_with_block(controller);
    } else {
      ER_INFO("%s aborted task '%s' (no block)",
              controller.GetId().c_str(),
              polled->name().c_str());
    }
    bool aborted = false;
    for (auto& h : penalty_handlers) {
      if (h->is_serving_penalty(controller)) {
        ER_ASSERT(!aborted,
                  "Controller serving penalties from more than one source");
        h->penalty_abort(controller);
        aborted = true;
        ER_INFO("%s aborted task '%s' while serving '%s' penalty",
                controller.GetId().c_str(),
                polled->name().c_str(),
                h->name().c_str());
      }
    } /* for(&h..) */
    return true;
  }

 private:
  void task_abort_with_block(T& controller) {
    /*
     * If the robot is currently right on the edge of a cache, we can't just
     * drop the block here, as it will overlap with the cache, and robots
     * will think that is accessible, but will not be able to vector to it
     * (not all 4 wheel sensors will report the color of a block). See #233.
     */
    bool conflict = false;
    m_map->cache_mtx().lock();
    for (auto& cache : m_map->caches()) {
      if (utils::block_drop_overlap_with_cache(
              controller.block(), cache, controller.position2D())) {
        conflict = true;
      }
    } /* for(cache..) */
    m_map->cache_mtx().unlock();

    /*
     * If the robot is currently right on the edge of the nest, we can't just
     * drop the block in the nest, as it will not be processed as a normal
     * \ref block_nest_drop, and will be discoverable by a robot via LOS but
     * not able to be acquired, as its color is hidden by that of the nest.
     *
     * If the robot is really close to a wall, then dropping a block may make
     * it inaccessible to future robots trying to reach it, due to obstacle
     * avoidance kicking in. This can result in an endless loop if said block
     * is the only one a robot knows about (see #242).
     */
    if (utils::block_drop_overlap_with_nest(
            controller.block(), m_map->nest(), controller.position2D()) ||
        utils::block_drop_near_arena_boundary(
            *m_map, controller.block(), controller.position2D())) {
      conflict = true;
    }
    perform_block_drop(controller, conflict);
  }

  void perform_block_drop(T& controller, bool drop_conflict) {
    /*
     * The robot owns a unique copy of a block originally from the arena, so we
     * need to look it up rather than implicitly converting its unique_ptr to a
     * shared_ptr and distributing it--this will cause lots of problems later.
     * This needs to be *BEFORE* releasing the robot's owned block.
     *
     * Holding the block mutex here is not necessary.
     */
    auto it = std::find_if(m_map->blocks().begin(),
                           m_map->blocks().end(),
                           [&](const auto& b) {
                             return controller.block()->id() == b->id();
                           });
    ER_ASSERT(m_map->blocks().end() != it,
              "Robot block%d not found in arena map blocks",
              controller.block()->id());

    events::free_block_drop_visitor drop_op(
        *it,
        rmath::dvec2uvec(controller.position2D(), m_map->grid_resolution().v()),
        m_map->grid_resolution(),
        true);

    m_map->block_mtx().lock();
    m_map->grid_mtx().lock();

    if (!drop_conflict) {
      drop_op.visit(*m_map);
    } else {
      m_map->distribute_single_block(*it);
    }
    m_map->grid_mtx().unlock();
    m_map->block_mtx().unlock();

    drop_op.visit(controller);
    m_floor->SetChanged();
  } /* perform_block_drop() */

  /* clang-format off */
  ds::arena_map* const       m_map;
  argos::CFloorEntity* const m_floor;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_ */
