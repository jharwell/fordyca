/**
 * \file dpo_controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/d0/dpo_controller_repository.hpp"

#include "fordyca/controller/config/block_sel/block_sel_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_controller_repository::dpo_controller_repository(void) {
  parser_register<block_sel::block_sel_matrix_parser,
                  block_sel::block_sel_matrix_config>(
      block_sel::block_sel_matrix_parser::kXMLRoot);
}

NS_END(d0, config, controller, fordyca);
