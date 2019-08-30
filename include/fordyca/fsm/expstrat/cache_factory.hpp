/**
 * @file cache_factory.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_CACHE_FACTORY_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_CACHE_FACTORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/factory/factory.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_factory :
    public rpfactory::releasing_factory<foraging_expstrat,
                                       const foraging_expstrat::params*> {
 public:
  static constexpr char kCRW[] = "CRW";
  static constexpr char kLikelihoodSearch[] = "likelihood_search";
  static constexpr char kUtilitySearch[] = "utility_search";
  static constexpr char kLEDTaxisSearch[] = "ledtaxis_search";

  cache_factory(void);
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_CACHE_FACTORY_HPP_ */
