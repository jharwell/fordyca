/**
 * \file d1/controller_repository.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/repository.hpp"

#include "fordyca/controller/config/d0/mdpo_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class controller_repository
 * \ingroup controller config d1
 *
 * \brief Collection of all parameter parsers and parse results needed
 * by d1 controllers.
 */
class controller_repository: public d0::mdpo_controller_repository,
                             public cta::config::xml::repository {
 public:
  controller_repository(void) RCPPSW_COLD;
};

NS_END(d1, config, controller, fordyca);
