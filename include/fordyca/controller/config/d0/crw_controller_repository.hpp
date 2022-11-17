/**
 * \file crw_controller_repository.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
NS_START(fordyca, controller, config, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw_controller_repository
 * \ingroup controller config d0
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * \ref crw_controller.
 */
class crw_controller_repository: public foraging_controller_repository {
 public:
  crw_controller_repository();
};

NS_END(d0, config, controller, fordyca);

