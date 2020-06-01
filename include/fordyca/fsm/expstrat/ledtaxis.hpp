/**
 * \file ledtaxis.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ledtaxis
 * \ingroup fsm expstrat
 *
 * \brief Assumes that the target entity type isequipped with an LED entity that
 * robots can detect with their blob camera. Performs phototaxis towards the
 * source of the signal if one is found, and performs phototaxis to it.
 */
class ledtaxis : public foraging_expstrat,
                 public rer::client<ledtaxis> {
 public:
  explicit ledtaxis(const foraging_expstrat::params* const c_params,
                    rmath::rng* rng)
      : ledtaxis(c_params->saa,
                 c_params->ledtaxis_target,
                 rng) {}
  ledtaxis(crfootbot::footbot_saa_subsystem* saa,
           const rutils::color& target,
           rmath::rng* rng);

  ~ledtaxis(void) override = default;
  ledtaxis(const ledtaxis&) = delete;
  ledtaxis& operator=(const ledtaxis&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final {
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
  std::unique_ptr<csexpstrat::base_expstrat> clone(void) const override {
    return nullptr; /* Should not be a top level explore behavior */
  }

  const rutils::color& target(void) const { return m_target; }

 private:
  /**
   * \brief The tolerance within which a robot's location has to be in order to
   * be considered having arrived at the taxis' source.
   */
  static constexpr double kARRIVAL_TOL = 1.0;

  /* clang-format off */
  mutable bool            m_task_running{false};
  rutils::color           m_target;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_HPP_ */
