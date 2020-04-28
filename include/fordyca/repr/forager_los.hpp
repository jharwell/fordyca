/**
 * \file forager_los.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_REPR_FORAGER_LOS_HPP_
#define INCLUDE_FORDYCA_REPR_FORAGER_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/los2D.hpp"
#include "cosm/ds/entity_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class forager_los
 * \ingroup repr
 *
 * \brief A line of sight for foraging applications, which computes the lists of
 * blocks and/or caches present in the LOS on request.
 *
 * The line of sight itself is meant to be a read-only view of part of the
 * arena, but it also exposes non-const access to the blocks and caches within
 * that part of the arena by necessity for event processing.
 */
class forager_los final : public crepr::los2D,
                          public rer::client<forager_los> {
 public:
  explicit forager_los(const const_grid_view& c_view)
      : los2D(c_view),
        ER_CLIENT_INIT("fordyca.repr.forager_los") {}

  /**
   * \brief Get the list of blocks currently in the LOS.
   *
   * This has to return a \ref cds::entity_vector, because we cannot generically
   * know if there are only 2D blocks, only 3D blocks, or a mix, currently in
   * the arena, so we have to do the generic thing. As long as a derived project
   * uses exclusively 2D or 3D blocks, and doesn't mix them, then this is a zero
   * cost abstraction, because it can be casted away with static_cast at compile
   * time.
   */
  cds::entity_vector blocks(void) const;
  cads::bcache_vectorno caches(void) const;
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_FORAGER_LOS_HPP_ */
