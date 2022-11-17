/**
 * \file cache_factory.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/cache_factory.hpp"

#include "fordyca/strategy/explore/crw_adaptor.hpp"
#include "fordyca/strategy/explore/ledtaxis_cache_search.hpp"
#include "fordyca/strategy/explore/likelihood_cache_search.hpp"
#include "fordyca/strategy/explore/utility_cache_search.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_factory::cache_factory(void) {
  register_type<crw_adaptor>(kCRW);
  register_type<likelihood_cache_search>(kLikelihoodSearch);
  register_type<utility_cache_search>(kUtilitySearch);
  register_type<ledtaxis_cache_search>(kLEDTaxisSearch);
}

NS_END(exploration, strategy, fordyca);
