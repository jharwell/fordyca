/**
 * @file existing_cache_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/tv/tv_manager.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
namespace ta = rcppsw::task_allocation;
namespace er = rcppsw::er;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class existing_cache_block_drop_interactor
 * @ingroup support
 *
 * @brief Handles a robot's (possible) \ref cached_block_drop event for existing
 * caches on a given timestep.
 */
template <typename T>
class existing_cache_block_drop_interactor
    : public er::client<existing_cache_block_drop_interactor<T>> {
 public:
  existing_cache_block_drop_interactor(ds::arena_map* const map_in,
                                       tv::tv_manager* tv_manager)
      : ER_CLIENT_INIT("fordyca.support.existing_cache_block_drop_interactor"),
        m_map(map_in),
        m_penalty_handler(tv_manager->penalty_handler<T>(
            tv::cache_op_src::kSrcExistingCacheDrop)) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  existing_cache_block_drop_interactor(
      const existing_cache_block_drop_interactor& other) = default;
  existing_cache_block_drop_interactor& operator=(
      const existing_cache_block_drop_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->penalty_satisfied(controller, timestep)) {
        finish_cache_block_drop(controller);
      }
    } else {
      m_penalty_handler->penalty_init(controller,
                                      tv::cache_op_src::kSrcExistingCacheDrop,
                                      timestep);
    }
  }

 private:
  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  void finish_cache_block_drop(T& controller) {
    const tv::temporal_penalty<T>& p = m_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<events::existing_cache_interactor*>(
                             controller.current_task()),
              "Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
                  tv::acquisition_goal_type::kExistingCache ==
                      controller.current_task()->acquisition_goal(),
              "Controller not waiting for cache block drop");

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet.
     *
     * This results in a \ref cached_block_drop with a pointer to a cache that
     * has already been destructed, and a segfault. See #247.
     *
     * Furthermore, it is also possible that while a robot is serving its pickup
     * penalty that the destination cache disappears AND then is re-created by
     * the arena or another robot dropping a block nearby. This does not appear
     * to be causing an error right now, but very well might in the future, so
     * we check that the ID of the cache we are sitting in/on when we finish
     * serving our penalty is the same as the one the penalty was originally
     * initialized with (not just checking if it is not -1).
     */
    int cache_id = loop_utils::robot_on_cache(controller, *m_map);

    if (p.id() != cache_id) {
      ER_WARN("%s cannot drop in cache%d: No such cache",
              controller.GetId().c_str(),
              p.id());
      events::cache_vanished vanished(p.id());

      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      perform_cache_block_drop(controller, p);
    }
    m_penalty_handler->remove(p);
    ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving cache penalty");
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_cache_block_drop(T& controller,
                                const tv::temporal_penalty<T>& penalty) {
    auto it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == penalty.id(); });
    ER_ASSERT(it != m_map->caches().end(),
              "Cache%d from penalty does not exist",
              penalty.id());
    events::cache_block_drop drop_op(controller.block(),
                                     *it,
                                     m_map->grid_resolution());
    (*it)->penalty_served(penalty.penalty());

    /* Update arena map state due to a cache drop */
    m_map->accept(drop_op);
    controller.visitor::template visitable_any<T>::accept(drop_op);
  }

  /* clang-format off */
  ds::arena_map* const                  m_map;
  tv::cache_op_penalty_handler<T>*const m_penalty_handler;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_ */
