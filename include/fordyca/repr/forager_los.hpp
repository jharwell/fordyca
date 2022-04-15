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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/grid2D_los.hpp"

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
class forager_los final : public crepr::grid2D_los,
                          public rer::client<forager_los> {
 public:
  forager_los(const rtypes::type_uuid& c_id,
              const grid_view_type& c_view,
              const rtypes::discretize_ratio& c_resolution);

  /**
   * \brief Get the list of blocks currently in the LOS.
   */
  cds::block3D_vectorno blocks(void) const;
  cads::bcache_vectorno caches(void) const;
};

NS_END(repr, fordyca);
