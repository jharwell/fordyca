/**
 * @file factory.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/fsm/expstrat/factory.hpp"
#include "fordyca/fsm/expstrat/crw.hpp"
#include "fordyca/fsm/expstrat/localized_block_search.hpp"
#include "fordyca/fsm/expstrat/localized_cache_search.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
constexpr char factory::kCRWBlock[];
constexpr char factory::kCRWCache[];
constexpr char factory::kLocalizedSearchBlock[];
constexpr char factory::kLocalizedSearchCache[];

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
factory::factory(void) {
  register_type<crw>(kCRWBlock);
  register_type<crw>(kCRWCache);
  register_type<localized_cache_search>(kLocalizedSearchCache);
  register_type<localized_block_search>(kLocalizedSearchBlock);
}

NS_END(expstrat, fsm, fordyca);
