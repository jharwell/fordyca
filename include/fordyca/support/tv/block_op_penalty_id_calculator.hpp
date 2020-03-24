/**
 * \file block_op_penalty_id_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_ID_CALCULATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_ID_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/support/tv/block_op_src.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::controller {
class foraging_controller;
} /* namespace fordyca::controller */

NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_op_penalty_id_calculator
 * \ingroup support tv
 *
 * \brief Extents \ref ctv::penalty_id_calculator with additional
 * FORDYCA-specific ways to calculate the ID that should be associated with a
 * particular temporal penalty.
 */
class block_op_penalty_id_calculator : public rer::client<block_op_penalty_id_calculator>,
                                       public rpdecorator::decorator<cforaging::tv::penalty_id_calculator> {
 public:
  explicit block_op_penalty_id_calculator(const carena::arena_map* map)
      : ER_CLIENT_INIT("fordyca.support.tv.block_op_penalty_id_calculator"),
        mc_map(map) {}

  /* Not copy constructable/assignable by default */
  block_op_penalty_id_calculator(const block_op_penalty_id_calculator&) = delete;
  const block_op_penalty_id_calculator& operator=(const block_op_penalty_id_calculator&) = delete;

  rtypes::type_uuid operator()(const controller::foraging_controller& controller,
                               block_op_src src) const;

 private:
  /* clang-formatoff */
  const carena::arena_map* mc_map;
  /* clang-format on */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_ID_CALCULATOR_HPP_ */
