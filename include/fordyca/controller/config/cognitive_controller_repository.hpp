/**
 * \file cognitive_controller_repository.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/foraging_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cognitive_controller_repository
 * \ingroup controller config
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref cognitive_controller.
 */
class cognitive_controller_repository: public virtual foraging_controller_repository {
 public:
  cognitive_controller_repository(void) RCPPSW_COLD;
};

NS_END(config, controller, fordyca);
