/**
 * \file argos_swarm_manager_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/argos/config/xml/swarm_manager_repository.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_swarm_manager_repository
 * \ingroup argos support config
 *
 * \brief Extra loop function XML parsers/results specific to the FORDYCA
 * project.
 */
class argos_swarm_manager_repository : public cpargos::config::xml::swarm_manager_repository {
 public:
  argos_swarm_manager_repository(void) noexcept RCPPSW_COLD;
};

NS_END(config, support, argos, fordyca);

