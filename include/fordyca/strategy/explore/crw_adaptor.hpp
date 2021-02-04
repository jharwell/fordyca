/**
 * \file crw_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_STRATEGY_EXPLORE_CRW_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_STRATEGY_EXPLORE_CRW_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/spatial/strategy/crw.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw_adaptor
 * \ingroup strategy explore
 *
 * \brief Adaptor for the CRW exploration strategy to enable it as an
 * exploration factory output within FORDYCA. Uses the decorator pattern on the
 * parent CRW class in order to avoid diamond inheritance with \ref
 * csstrategy::base_strategy.
 */
class crw_adaptor final : public foraging_strategy,
                          public rpdecorator::decorator<csstrategy::crw> {
 public:
  crw_adaptor(const foraging_strategy::params* c_params, rmath::rng* rng);
  crw_adaptor(crfootbot::footbot_saa_subsystem* saa, rmath::rng* rng);

  ~crw_adaptor(void) override = default;
  crw_adaptor(const crw_adaptor&) = delete;
  crw_adaptor& operator=(const crw_adaptor&) = delete;

  /* interference metrics */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exp_interference, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(entered_interference, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exited_interference, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_duration, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_loc3D, decoratee(), const);

  RCPPSW_WRAP_DECLDEF_OVERRIDE(task_reset, decoratee());
  RCPPSW_WRAP_DECLDEF_OVERRIDE(task_running, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(task_finished, decoratee(), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(task_execute, decoratee());
  void task_start(cta::taskable_argument* arg) override { decoratee().task_start(arg); }

  /* prototype overrides */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(clone, decoratee(), const);
};

NS_END(explore, strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_EXPLORE_CRW_ADAPTOR_HPP_ */
