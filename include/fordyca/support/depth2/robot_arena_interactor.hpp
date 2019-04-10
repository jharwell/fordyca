/**
 * @file depth2/robot_arena_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth2/cache_site_block_drop_interactor.hpp"
#include "fordyca/support/depth2/new_cache_block_drop_interactor.hpp"
#include "fordyca/support/task_abort_interactor.hpp"
#include "fordyca/support/cached_block_pickup_interactor.hpp"
#include "fordyca/support/existing_cache_block_drop_interactor.hpp"
#include "fordyca/support/free_block_pickup_interactor.hpp"
#include "fordyca/support/nest_block_drop_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace ta = rcppsw::task_allocation;
class dynamic_cache_manager;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class robot_arena_interactor
 * @ingroup support depth2
 *
 * @brief Handles a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up from/dropping a block in a cache.
 * - Subjecting robots using caches to a penalty on cache drop/pickup.
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 * - Free block drop due to task abort.
 * - Creating a new cache.
 */
template <typename T>
class robot_arena_interactor : public er::client<robot_arena_interactor<T>> {
 public:
  using controller_type = T;
  robot_arena_interactor(ds::arena_map* const map,
                         depth0::depth0_metrics_aggregator *const metrics_agg,
                         argos::CFloorEntity* const floor,
                         tv::tv_manager* const tv_manager,
                         dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.robot_arena_interactor"),
        m_tv_manager(tv_manager),
        m_free_pickup_interactor(map, floor, tv_manager),
        m_nest_drop_interactor(map, metrics_agg, floor, tv_manager),
        m_task_abort_interactor(map, floor),
        m_cached_pickup_interactor(map, floor, tv_manager, cache_manager),
        m_existing_cache_drop_interactor(map, tv_manager),
        m_cache_site_drop_interactor(map,
                                     floor,
                                     tv_manager,
                                     cache_manager),
        m_new_cache_drop_interactor(map,
                                    floor,
                                    tv_manager,
                                    cache_manager) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  robot_arena_interactor(const robot_arena_interactor& other) = default;
  robot_arena_interactor& operator=(
      const robot_arena_interactor& other) = delete;


  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   *
   * @return \c TRUE if a block was dropped to create a new cache, \c FALSE
   * otherwise.
   */
  bool operator()(T& controller, uint timestep) {
    if (m_task_abort_interactor(controller,
                                m_tv_manager->template all_penalty_handlers<T>())) {
      /*
       * This needs to be here, rather than in each robot's control step
       * function, in order to avoid triggering erroneous handling of an aborted
       * task in the loop functions when the executive has not aborted the newly
       * allocated task. See #532.
       */
      controller.task_aborted(false);
      return false;
    }

    if (controller.is_carrying_block()) {
      m_nest_drop_interactor(controller, timestep);
      m_existing_cache_drop_interactor(controller, timestep);
      m_cache_site_drop_interactor(controller, timestep);
      return m_new_cache_drop_interactor(controller, timestep);
    } else { /* The foot-bot has no block item */
      m_free_pickup_interactor(controller, timestep);
      m_cached_pickup_interactor(controller, timestep);
      return false;
    }
  }


 private:
  /* clang-format off */
  tv::tv_manager* const                m_tv_manager;

  free_block_pickup_interactor<T>         m_free_pickup_interactor;
  nest_block_drop_interactor<T>           m_nest_drop_interactor;
  task_abort_interactor<T>                m_task_abort_interactor;
  cached_block_pickup_interactor<T>       m_cached_pickup_interactor;
  existing_cache_block_drop_interactor<T> m_existing_cache_drop_interactor;
  cache_site_block_drop_interactor<T>     m_cache_site_drop_interactor;
  new_cache_block_drop_interactor<T>      m_new_cache_drop_interactor;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_ */
