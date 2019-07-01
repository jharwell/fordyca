/**
 * @file interactor_status.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_INTERACTOR_STATUS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_INTERACTOR_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
enum class interactor_status {
  ekNoEvent           = 1 << 0,
  ekNestBlockDrop     = 1 << 1,
  ekFreeBlockPickup   = 1 << 2,
  ekNewCacheBlockDrop = 1 << 3,
  ekTaskAbort         = 1 << 4,
  ekCacheDepletion    = 1 << 5,
  ekFreeBlockDrop     = 1 << 6
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_INTERACTOR_STATUS_HPP_ */
