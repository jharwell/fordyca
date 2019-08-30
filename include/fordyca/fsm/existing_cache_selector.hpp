/**
 * @file existing_cache_selector.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_EXISTING_CACHE_SELECTOR_HPP_
#define INCLUDE_FORDYCA_FSM_EXISTING_CACHE_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/ds/dp_cache_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class cache_sel_matrix;
} /* namespace controller */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class existing_cache_selector
 * @ingroup fordyca fsm
 *
 * @brief Selects from among known caches (which are presumed to still exist at
 * this point, although that may not be true as a robot's knowledge of the arena
 * is imperfect), using an internal utility function.
 */
class existing_cache_selector : public rer::client<existing_cache_selector> {
 public:
  existing_cache_selector(bool is_pickup,
                          const controller::cache_sel_matrix* matrix,
                          const ds::dp_cache_map* cache_map);

  ~existing_cache_selector(void) override = default;
  existing_cache_selector& operator=(const existing_cache_selector& other) =
      delete;
  existing_cache_selector(const existing_cache_selector& other) = delete;

  /**
   * @brief Given a list of existing caches that a robot knows about (i.e. have
   * not faded into an unknown state), compute which is the "best", for use in
   * deciding which cache to go to and attempt to pickup from.
   *
   * @return The "best" existing cache.
   */
  boost::optional<ds::dp_cache_map::value_type> operator()(
      const ds::dp_cache_map& existing_caches,
      const rmath::vector2d& position,
      rtypes::timestep t);

 private:
  /**
   * @brief Determine if the specified cache is excluded from being considered
   * for selection because:
   *
   * - The robot is currently inside it.
   * - It is on the exception list.
   *
   * @return \c TRUE if the cache should be excluded, \c FALSE otherwise.
   */
  bool cache_is_excluded(const rmath::vector2d& position,
                         const repr::base_cache* cache) const;

  /* clang-format off */
  const bool                                mc_is_pickup;
  const controller::cache_sel_matrix* const mc_matrix;
  const ds::dp_cache_map* const             mc_cache_map;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXISTING_CACHE_SELECTOR_HPP_ */
