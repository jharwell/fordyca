/**
 * \file mdpo_controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/d0/dpo_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mdpo_controller_repository
 * \ingroup controller config d0
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref mdpo_controller.
 */
class mdpo_controller_repository: public virtual dpo_controller_repository {
 public:
  mdpo_controller_repository(void) = default;
};

NS_END(d0, config, controller, fordyca);
