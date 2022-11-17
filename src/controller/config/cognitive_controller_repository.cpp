/**
 * \file cognitive_controller_repository.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/cognitive_controller_repository.hpp"

#include "fordyca/subsystem/perception/config/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cognitive_controller_repository::cognitive_controller_repository(void) {
  parser_register<fspconfig::perception_parser, fspconfig::perception_config>(
      fspconfig::perception_parser::kXMLRoot);
}

NS_END(config, controller, fordyca);
