/**
 * \file fsm_ro_params.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_FSM_RO_PARAMS_HPP_
#define INCLUDE_FORDYCA_FSM_FSM_RO_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/config/exploration_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller::cognitive {
class block_sel_matrix;
class cache_sel_matrix;
} // namespace controller::cognitive

namespace ds {
class dpo_store;
} /* namespace ds */

NS_START(fsm);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct fsm_ro_params
 * \ingroup fsm
 *
 * \brief Contains all parameters for FSM initialization that will be read-only
 * by the FSM at run-time; not all FSMs need all members.
 */
struct fsm_ro_params {
  const controller::cognitive::block_sel_matrix* bsel_matrix;
  const controller::cognitive::cache_sel_matrix* csel_matrix;
  const ds::dpo_store* store;
  const fordyca::config::exploration_config exp_config;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_FSM_RO_PARAMS_HPP_ */
