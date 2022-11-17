/**
 * \file perception_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "cosm/subsystem/perception/config/rlos_config.hpp"

#include "fordyca/subsystem/perception/config/dpo_config.hpp"
#include "fordyca/subsystem/perception/config/mdpo_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct perception_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for robot perception.
 */
struct perception_config final : public rconfig::base_config {
  std::string type{""};

  dpo_config dpo {};
  mdpo_config mdpo {};
};

NS_END(config, perception, subsystem, fordyca);

