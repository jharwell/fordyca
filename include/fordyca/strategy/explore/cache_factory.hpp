/**
 * \file cache_factory.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_factory
 * \ingroup strategy explore
 *
 * \brief Factory for creating cache exploration strategies.
 */
class cache_factory :
    public rpfactory::releasing_factory<fstrategy::foraging_strategy,
                                        std::string, /* key type */
                                        const fstrategy::strategy_params*,
                                        rmath::rng*> {
 public:
  static inline const std::string kCRW = "CRW";
  static inline const std::string kLikelihoodSearch = "likelihood_search";
  static inline const std::string kUtilitySearch = "utility_search";
  static inline const std::string kLEDTaxisSearch = "ledtaxis_search";

  cache_factory(void);
};

NS_END(explore, strategy, fordyca);
