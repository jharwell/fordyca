/**
 * \file block_op_penalty_id_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/argos/support/tv/block_op_src.hpp"
#include "fordyca/argos/support/tv/op_filter_result.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

namespace fordyca::controller {
class foraging_controller;
} /* namespace fordyca::controller */

NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_op_penalty_id_calculator
 * \ingroup argos support tv
 *
 * \brief Extents \ref ctv::penalty_id_calculator with additional
 * FORDYCA-specific ways to calculate the ID that should be associated with a
 * particular temporal penalty.
 */
class block_op_penalty_id_calculator : public rer::client<block_op_penalty_id_calculator>,
                                       public rpdecorator::decorator<cforaging::tv::penalty_id_calculator> {
 public:
  explicit block_op_penalty_id_calculator(const carena::caching_arena_map* map);

  /* Not copy constructable/assignable by default */
  block_op_penalty_id_calculator(const block_op_penalty_id_calculator&) = delete;
  const block_op_penalty_id_calculator& operator=(const block_op_penalty_id_calculator&) = delete;

  rtypes::type_uuid operator()(const controller::foraging_controller& controller,
                               const block_op_src& src,
                               const op_filter_result& filter) const;

 private:
  /* clang-formatoff */
  const carena::caching_arena_map* mc_map;
  /* clang-format on */
};

NS_END(tv, support, argos, fordyca);

