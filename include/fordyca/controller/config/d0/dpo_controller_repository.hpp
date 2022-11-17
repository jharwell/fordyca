/**
 * \file dpo_controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/cognitive_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_controller_repository
 * \ingroup controller config d0
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref dpo_controller.
 */
class dpo_controller_repository: public virtual cognitive_controller_repository {
 public:
  dpo_controller_repository(void) RCPPSW_COLD;
};

NS_END(d0, config, controller, fordyca);
