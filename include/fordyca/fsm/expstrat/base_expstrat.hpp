/**
 * @file base_expstrat.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/ta/taskable.hpp"
#include "fordyca/config/exploration_config.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller {
class saa_subsystem;
} /* namespace controller */
namespace ds {
class dpo_store;
} /* namespace ds */

NS_START(fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_expstrat
 * @ingroup fordyca fsm expstrat
 *
 * @brief Base class for different exploration behaviors that robots can exhibit
 * when looking for stuff.
 */
class base_expstrat : public metrics::fsm::collision_metrics,
                              public rta::taskable,
                              public rprototype::clonable<base_expstrat> {
 public:
  struct params {
    params(controller::saa_subsystem* saa_in, ds::dpo_store* store_in)
        : saa(saa_in),
          store(store_in) {}
    controller::saa_subsystem*  saa;
    ds::dpo_store*              store;
  };

  explicit base_expstrat(const params* const c_params)
      : base_expstrat(c_params->saa) {}

  ~base_expstrat(void) override = default;

  base_expstrat(const base_expstrat&) = delete;
  base_expstrat& operator=(const base_expstrat&) = delete;

 protected:
  explicit base_expstrat(controller::saa_subsystem* saa)
      : m_saa(saa) {}

  controller::saa_subsystem* saa_subsystem(void) const { return m_saa; }
  controller::saa_subsystem* saa_subsystem(void) { return m_saa; }

 private:
  /* clang-format off */
  controller::saa_subsystem* m_saa;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_ */
