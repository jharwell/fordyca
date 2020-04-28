/**
 * \file dpo_controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/config/depth0/dpo_controller_repository.hpp"

#include "fordyca/config/block_sel/block_sel_matrix_parser.hpp"
#include "cosm/controller/config/perception/xml/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_controller_repository::dpo_controller_repository(void) {
  parser_register<block_sel::block_sel_matrix_parser,
                  block_sel::block_sel_matrix_config>(
      block_sel::block_sel_matrix_parser::kXMLRoot);
  parser_register<ccontconfig::perception::xml::perception_parser,
                  ccontconfig::perception::perception_config>(
                      ccontconfig::perception::xml::perception_parser::kXMLRoot);
}

NS_END(depth0, config, fordyca);
