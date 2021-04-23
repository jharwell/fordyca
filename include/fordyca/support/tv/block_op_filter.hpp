/**
 * \file block_op_filter.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <boost/optional.hpp>

#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/support/tv/block_op_src.hpp"
#include "fordyca/support/tv/op_filter_result.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/support/cache_prox_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class block_op_filter
 * \ingroup support tv
 *
 * \brief The filter for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
class block_op_filter : public rer::client<block_op_filter> {
 public:
  explicit block_op_filter(const carena::caching_arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.tv.block_op_filter"),
        mc_map(map) {}

  ~block_op_filter(void) override = default;
  block_op_filter& operator=(const block_op_filter&) = delete;
  block_op_filter(const block_op_filter&) = delete;

  /**
   * \brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * \return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  template <typename TController>
  op_filter_result operator()(const TController& controller,
                              block_op_src src,
                              boost::optional<rtypes::spatial_dist> cache_prox) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        return free_pickup_filter(controller);
      case block_op_src::ekNEST_DROP:
        return nest_drop_filter(controller);
      case block_op_src::ekCACHE_SITE_DROP:
        ER_ASSERT(cache_prox && *cache_prox > 0.0,
                  "Cache proximity distance not specified for cache site drop");
        return cache_site_drop_filter(controller, *cache_prox);
      case block_op_src::ekNEW_CACHE_DROP:
        ER_ASSERT(cache_prox && *cache_prox > 0.0,
                  "Cache proximity distance not specified for new cache drop");
        return new_cache_drop_filter(controller, *cache_prox);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", static_cast<int>(src));
    } /* switch() */
    return op_filter_result{};
  }

 private:
  /**
   * \brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   */
  template <typename TController>
  op_filter_result free_pickup_filter(const TController& controller) const {
    op_filter_result result;
    if (!(controller.goal_acquired() &&
          fsm::foraging_acq_goal::ekBLOCK == controller.acquisition_goal())) {
      result.status = op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else {
      /*
       * OK to lock around this calculation, because relatively few robots will
       * have the correct internal state for a free block pickup each timestep,
       * and we don't need exclusive access to the arena map at this point--only
       * a guarantee that the blocks/caches arrays will not be modified while we
       * are checking them.
       */
      mc_map->lock_rd(mc_map->cache_mtx());
      mc_map->lock_rd(mc_map->block_mtx());
      auto block_id = mc_map->robot_on_block(controller.rpos2D(),
                                            controller.entity_acquired_id());
      mc_map->unlock_rd(mc_map->block_mtx());
      mc_map->unlock_rd(mc_map->cache_mtx());

      if (rtypes::constants::kNoUUID == block_id) {
        result.status = op_filter_status::ekROBOT_NOT_ON_BLOCK;
      } else {
        result.status = op_filter_status::ekSATISFIED;
        result.id = block_id;
      }
    }
     return result;
  }

  /**
   * \brief Filter out spurious penalty initializations for nest block drop
   * (i.e. controller not ready/not intending to drop a block in the nest).
   */
  template <typename TController>
  op_filter_result nest_drop_filter(const TController& controller) const {
    op_filter_result result;
    if (!(controller.in_nest() && controller.goal_acquired() &&
          fsm::foraging_transport_goal::ekNEST == controller.block_transport_goal())) {
      result.status = op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else {
      result.status = op_filter_status::ekSATISFIED;
    }
    return result;
  }

  /**
   * \brief Filter out spurious penalty initializations for cache site drop
   * (i.e. controller not ready/not intending to drop a block), or another
   * block/cache is too close.
   */
  template <typename TController>
  op_filter_result cache_site_drop_filter(const TController& controller,
                                          const rtypes::spatial_dist& cache_prox) const {
    op_filter_result result;
    if (!(controller.goal_acquired() &&
          fsm::foraging_acq_goal::ekCACHE_SITE == controller.acquisition_goal() &&
          fsm::foraging_transport_goal::ekCACHE_SITE == controller.block_transport_goal())) {
      result.status = op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else {
      cache_prox_checker checker(mc_map, cache_prox);
      auto prox = checker.check(controller);

      if (rtypes::constants::kNoUUID != prox.id) {
        result.status = op_filter_status::ekCACHE_PROXIMITY;
        result.id = prox.id;
      } else {
        result.status = op_filter_status::ekSATISFIED;
      }
    }

    return result;
  }

  /**
   * \brief Filter out spurious penalty initializations for new cache drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to another cache to do a free block drop at the chosen site.
   */
  template <typename TController>
  op_filter_result new_cache_drop_filter(const TController& controller,
                                         const rtypes::spatial_dist& cache_prox) const {
    op_filter_result result;
    if (!(controller.goal_acquired() &&
          fsm::foraging_acq_goal::ekNEW_CACHE == controller.acquisition_goal() &&
          fsm::foraging_transport_goal::ekNEW_CACHE == controller.block_transport_goal())) {
      result.status = op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else {
      cache_prox_checker checker(mc_map, cache_prox);
      auto prox = checker.check(controller);

      if (rtypes::constants::kNoUUID != prox.id) {
        result.status = op_filter_status::ekCACHE_PROXIMITY;
        result.id = prox.id;
      } else {
        result.status = op_filter_status::ekSATISFIED;
      }
    }
    return result;
  }

  /* clang-format off */
  const carena::caching_arena_map* mc_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_ */
