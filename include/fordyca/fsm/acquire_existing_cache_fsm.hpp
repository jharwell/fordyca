/**
 * @file acquire_existing_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>
#include <random>
#include <utility>

#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class cache_sel_matrix;
}
namespace repr {
class cache;
}
namespace ds {
class dpo_store;
}

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_existing_cache_fsm
 * @ingroup fordyca fsm depth1
 *
 * @brief Acquire an existing cache within the arena. Once such a cache has been
 * acquired (always by vectoring), it signals that it has completed its task.
 */
class acquire_existing_cache_fsm
    : public rer::client<acquire_existing_cache_fsm>,
      public acquire_goal_fsm {
 public:
  /**
   * @param matrix The matrix of cache selection info.
   * @param saa Handle to sensing/actuation subsystem.
   * @param store Store of known objects in the arena.
   * @param behavior The exploration behavior to use when acquiring a cache.
   * @param for_pickup Are we acquiring a cache for pickup or block drop?
   */
  acquire_existing_cache_fsm(const controller::cache_sel_matrix* matrix,
                             controller::saa_subsystem* saa,
                             ds::dpo_store* store,
                             std::unique_ptr<expstrat::base_expstrat> behavior,
                             bool for_pickup);

  ~acquire_existing_cache_fsm(void) override = default;

  acquire_existing_cache_fsm(const acquire_existing_cache_fsm&) = delete;
  acquire_existing_cache_fsm& operator=(const acquire_existing_cache_fsm&) =
      delete;

  void by_exploration_ok(bool b) { m_by_exploration_ok = b; }

 private:
  using acq_loc_type = std::pair<int, rmath::vector2d>;
  /*
   * See \ref acquire_goal_fsm for the purpose of these callbacks.
   */
  acq_goal_type acq_goal_internal(void) const;
  boost::optional<acquire_goal_fsm::candidate_type> existing_cache_select(void);
  bool candidates_exist(void) const;
  boost::optional<acq_loc_type> calc_acq_location(void);
  bool cache_acq_valid(const rmath::vector2d& loc, uint id);

  bool cache_acquired_cb(bool explore_result) const;

  /* clang-format off */
  const bool                                mc_for_pickup;
  const controller::cache_sel_matrix* const mc_matrix;
  const ds::dpo_store*      const           mc_store;

  /**
   * @brief Is it OK to acquire a cache via exploration? Usually you do not want
   * this because:
   *
   * - The cache we just acquired might have been one that was excluded from the
   *   list of eligible caches for acquisition.
   * - We might not have meant the criteria for block pickup from this cache
   *   yet.
   */
  bool                                      m_by_exploration_ok{false};

  /**
   * @brief Is the current cache acquisition valid? We need to save this when it
   * is computed, because it is computed using a callback from the \ref
   * acquire_goal_fsm and we get passed information we cannot get internally. We
   * save it so that if we happen to be exploring for a cache because all known
   * caches are invalid, we don't signal "goal acquired" via \ref
   * cache_exploration_term_cb() and acquire a cache erroneously.
   */
  bool                                      m_acq_valid{false};
  std::default_random_engine                m_rd;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_ */
