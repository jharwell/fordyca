/**
 * \file d2/controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/d1/controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class controller_repository
 * \ingroup controller config d2
 *
 * \brief Collection of all parameter parsers and parse results needed
 * by d2 controllers.
 */
class controller_repository: public d1::controller_repository {
 public:
  controller_repository(void) RCPPSW_COLD;
};

NS_END(d2, config, controller, fordyca);

