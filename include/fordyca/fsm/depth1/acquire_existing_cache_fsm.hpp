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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_EXISTING_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_EXISTING_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <argos3/core/utility/math/vector2.h>

#include "fordyca/fsm/depth1/base_acquire_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_existing_cache_fsm
 * @ingroup fsm depth1
 *
 * @brief Acquire an existing cache within the arena. Once such a cache has been
 * acquired (either a known existing cache or via random exploration), it
 * signals that it has completed its task.
 */
class acquire_existing_cache_fsm : public base_acquire_cache_fsm {
 public:
  acquire_existing_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      std::shared_ptr<const representation::perceived_arena_map> map);

  acquire_existing_cache_fsm(const acquire_existing_cache_fsm& fsm) = delete;
  acquire_existing_cache_fsm& operator=(const acquire_existing_cache_fsm& fsm) = delete;

  argos::CVector2 select_cache_for_acquisition(void) override;
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_EXISTING_CACHE_FSM_HPP_ */
