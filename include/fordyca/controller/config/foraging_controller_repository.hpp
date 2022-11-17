/**
 * \file foraging_controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/config/xml/base_controller_repository.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_controller_repository
 * \ingroup controller config
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * all foraging controllers.
 */
class foraging_controller_repository
    : public virtual ccontroller::config::xml::base_controller_repository {
 public:
  foraging_controller_repository(void) RCPPSW_COLD;
};

NS_END(config, controller, fordyca);
