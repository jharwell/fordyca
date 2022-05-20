/**
 * \file cache_factory.cpp
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
