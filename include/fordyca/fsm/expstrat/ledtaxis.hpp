/**
 * @file ledtaxis.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/expstrat/base_expstrat.hpp"
#include "fordyca/fsm/collision_tracker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ledtaxis
 * @ingroup fordyca fsm expstrat
 *
 * @brief Assumes that the target entity type isequipped with an LED entity that
 * robots can detect with their blob camera. Performs phototaxis towards the
 * source of the signal if one is found, and performs phototaxis to it.
 */
class ledtaxis : public base_expstrat,
                 public rer::client<ledtaxis> {
 public:
  explicit ledtaxis(const base_expstrat::params* const c_params)
      : ledtaxis(c_params->saa, c_params->ledtaxis_target) {}
  explicit ledtaxis(controller::saa_subsystem* saa,
                    const rutils::color& target)
      : base_expstrat(saa),
        ER_CLIENT_INIT("fordyca.fsm.expstrat.ledtaxis"),
        m_tracker(saa),
        m_target(target) {}

  ~ledtaxis(void) override = default;
  ledtaxis(const ledtaxis&) = delete;
  ledtaxis& operator=(const ledtaxis&) = delete;

  /* taskable overrides */
  void task_start(const rta::taskable_argument*) override final {
    m_task_running = true;
  }
  void task_reset(void) override final {
    m_task_running = false;
  }
  bool task_running(void) const override final {
    return m_task_running;
  }
  bool task_finished(void) const override final;
  void task_execute(void) override final;

  /* prototype overrides */
  std::unique_ptr<base_expstrat> clone(void) const override {
    return nullptr; /* Should not be a top level explore behavior */
  }

  const rutils::color& target(void) const { return m_target; }

 private:
  /**
   * @brief The tolerance within which a robot's location has to be in order to
   * be considered having arrived at the taxis' source.
   */
  static constexpr double kARRIVAL_TOL = 1.0;

  /* clang-format off */
  mutable bool      m_task_running{false};
  collision_tracker m_tracker;
  rutils::color     m_target;
  /* clang-format on */

 public:
  /* collision metrics */
  RCPPSW_DECLDEF_OVERRIDE_WRAP(in_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(entered_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(exited_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(collision_avoidance_duration, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(avoidance_loc, m_tracker, const)
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_ */