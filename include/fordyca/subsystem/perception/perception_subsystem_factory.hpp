/**
 * \file perception_subsystem_factory.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"
#include "fordyca/fordyca.hpp"

#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/config/perception_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_subsystem_factory
 * \ingroup strategy explore
 *
 * \brief Factory for creating \ref fsperception::foraging_perception_subsystem
 * derived objects.
 */
class perception_subsystem_factory :
    public rpfactory::releasing_factory<fsperception::foraging_perception_subsystem,
                                        std::string, /* key type */
                                        const config::perception_config*> {
 public:
  static inline const std::string kDPO = "dpo";
  static inline const std::string kMDPO = "mdpo";

  perception_subsystem_factory(void);
};

NS_END(perception, subsystem, fordyca);

