/**
 * \file cache_factory.hpp
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
