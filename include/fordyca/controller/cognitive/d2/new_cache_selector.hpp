/**
 * \file new_cache_selector.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(fordyca, controller, cognitive);
class cache_sel_matrix;
NS_START(d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class new_cache_selector
 * \ingroup controller cognitive d2
 *
 * \brief Selects from among "new" caches (which are the same as blocks in the
 * arena) which are presumed to still exist at this point, although that may not
 * be true as a robot's knowledge of the arena is imperfect).
 */
class new_cache_selector: public rer::client<new_cache_selector> {
 public:
  explicit new_cache_selector(const controller::cognitive::cache_sel_matrix* csel_matrix);

  ~new_cache_selector(void) override = default;
  new_cache_selector& operator=(const new_cache_selector&) = delete;
  new_cache_selector(const new_cache_selector&) = delete;

  /**
   * \brief Given a list of new caches that a robot knows about, compute which
   * is the "best", taking into account proximity to known caches alread in
   * existence.
   *
   * \return The "best" new cache.
   */
  const crepr::base_block3D* operator()(const fspds::dp_block_map& new_caches,
                                        const fspds::dp_cache_map& existing_caches,
                                        const rmath::vector2d& position) const;

 private:
  bool new_cache_is_excluded(const fspds::dp_cache_map& existing_caches,
                             const fspds::dp_block_map& blocks,
                             const crepr::base_block3D* new_cache) const;

  /* clang-format off */
  const controller::cognitive::cache_sel_matrix* const mc_matrix;
  /* clang-format on */
};

NS_END(cognitive, d2, controller, fordyca);

