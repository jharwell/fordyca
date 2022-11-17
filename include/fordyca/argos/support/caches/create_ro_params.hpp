/**
 * \file create_ro_params.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
