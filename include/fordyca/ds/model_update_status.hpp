/**
 * \file model_update_status.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_DS_MODEL_UPDATE_STATUS_HPP_
#define INCLUDE_FORDYCA_DS_MODEL_UPDATE_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Communicates the nature of an update made to a \ref
 * base_perception_model derived class as a result of processing a seen
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

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_MODEL_UPDATE_STATUS_HPP_ */
