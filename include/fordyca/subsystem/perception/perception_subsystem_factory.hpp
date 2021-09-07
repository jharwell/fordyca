/**
 * \file perception_subsystem_factory.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_SUBSYSTEM_FACTORY_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_SUBSYSTEM_FACTORY_HPP_

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

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_PERCEPTION_SUBSYSTEM_FACTORY_HPP_ */
