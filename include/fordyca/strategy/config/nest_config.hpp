/**
 * \file nest_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
  * Includes
******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/spatial/strategy/nest/config/acq_config.hpp"
#include "cosm/spatial/strategy/nest/config/exit_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct nest_config
  * \ingroup strategy config
  *
  * \brief Configuration for strategies related to things robots can do related
  * to the nest while foraging.
  */
struct nest_config final : public rconfig::base_config {
  cssnest::config::acq_config acq{};
  cssnest::config::exit_config exit{};
};

NS_END(config, strategy, fordyca);
