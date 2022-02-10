/**
 * \file strategy_params.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"

#include "cosm/spatial/fsm/fsm_params.hpp"

#include "fordyca/subsystem/perception/perception_fwd.hpp"
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::controller::cognitive {
class cache_sel_matrix;
class block_sel_matrix;
} // namespace controller::cognitive

NS_START(fordyca, strategy);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
  struct strategy_params {
    strategy_params(const csfsm::fsm_params* fsm_in,
           const controller::cognitive::block_sel_matrix* const bsel_matrix_in,
           const controller::cognitive::cache_sel_matrix* const csel_matrix_in,
           const fsperception::known_objects_accessor* accesor_in,
           const rutils::color& ledtaxis_target_in)
        : fsm(fsm_in),
          bsel_matrix(bsel_matrix_in),
          csel_matrix(csel_matrix_in),
          accessor(accesor_in),
          ledtaxis_target(ledtaxis_target_in) {}

    strategy_params(const strategy_params&) = delete;
    strategy_params& operator=(const strategy_params&) = delete;

    const csfsm::fsm_params* fsm;
    const controller::cognitive::block_sel_matrix* bsel_matrix;
    const controller::cognitive::cache_sel_matrix* csel_matrix;
    const fsperception::known_objects_accessor* accessor;
    rutils::color ledtaxis_target;
  };

NS_END(strategy, fordyca);

