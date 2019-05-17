/**
 * @file factory.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_FACTORY_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_FACTORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/factory/factory.hpp"
#include "fordyca/nsalias.hpp"
#include "fordyca/fsm/expstrat/base_expstrat.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class factory :
    public rfactory::releasing_factory<base_expstrat,
                                       const base_expstrat::params*> {
 public:
  static constexpr char kCRWCache[] = "CRW_cache";
  static constexpr char kCRWBlock[] = "CRW_block";
  static constexpr char kLocalizedSearchCache[] = "localized_search_cache";
  static constexpr char kLocalizedSearchBlock[] = "localized_search_block";

  factory(void);

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_FACTORY_HPP_ */
