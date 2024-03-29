/**
 * \file create_ro_params.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "rcppsw/types/timestep.hpp"

#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct create_ro_params
 * \ingroup argos support caches
 *
 * \brief Parameters for cache creation, to reduce # of arguments to
 * functions. Contains all parameters that will *NOT* change during creation.
 */
struct create_ro_params {
  /**
   * \brief Currently existing caches in the arena. For use in avoiding
   * overlaps during cache creation.
   */
  cads::acache_vectorno current_caches;

  /**
   * \brief The block clusters in the arena. For use in avoiding overlaps
   * during cache creation.
   */
  cfds::block3D_cluster_vectorro clusters;

  /**
   * \brief The current timestep.
   */
  rtypes::timestep t;
};

NS_END(caches, support, argos, fordyca);
