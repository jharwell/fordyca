/**
 * \file model_update_status.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \enum model_update_status
 * \ingroup subsystem perception
 *
 * \brief Communicates the nature of an update made to a \ref
 * foraging_perception_model derived class as a result of processing a seen
 * block/cache.
 */
enum class model_update_status {
  /**
   * \brief No changes were made to the model.
   */
  ekNO_CHANGE,

  /**
   * \brief Nothing about the block has changed but its density.
   */
  ekBLOCK_DENSITY_UPDATE,

  /**
   * \brief A new block was added to the model.
   */
  ekNEW_BLOCK_ADDED,

  /**
   * \brief An already tracked block was updated with a new location.
   */
  ekBLOCK_MOVED,

  /**
   * \brief A new cache was added to the model.
   */
  ekNEW_CACHE_ADDED,

  /**
   * \brief An already tracked block was updated (e.g., the # of blocks in it
   * changed).
   */
  ekCACHE_UPDATED
};

NS_END(perception, subsystem, fordyca);

