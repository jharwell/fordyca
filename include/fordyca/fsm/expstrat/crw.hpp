/**
 * \file crw.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_CRW_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_CRW_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "rcppsw/er/client.hpp"
#include "cosm/fsm/collision_tracker.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw
 * \ingroup fordyca fsm expstrat
 *
 * \brief Roam around using Correlated Random Walk looking for something until
 * you happen to stumble across it.
 */
class crw final : public foraging_expstrat,
                  public rer::client<crw> {
 public:
  crw(const fsm::expstrat::foraging_expstrat::params* const c_params,
      rmath::rng* rng);

  crw(crfootbot::footbot_saa_subsystem* saa, rmath::rng* rng);

  ~crw(void) override = default;
  crw(const crw&) = delete;
  crw& operator=(const crw&) = delete;

  /* taskable overrides */
  void task_start(const rta::taskable_argument*) override final {
    m_task_running = true;
  }
  void task_reset(void) override final { m_task_running = false; }
  bool task_running(void) const override final { return m_task_running; }

  /**
   * \brief Since we are exploring for something we don't know about, we will
   * never finish (stopping exploration is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;

    /* prototype overrides */
  std::unique_ptr<foraging_expstrat> clone(void) const override {
    return std::make_unique<crw>(saa(), rng());
  }

 private:
  /**
   * \brief Handle all logic for entering collision avoidance; derived classes
   * should only have to call this function whenever they detect an obstacle.
   *
   * \param timestep The current timestep.
   */
  void ca_enter(rtypes::timestep t);

  /**
   * \brief Handle all logic for exiting collision avoidance; derived classes
   * should only have to call this function whenever they no longer detect any
   * obstacles.
   *
   * \param timestep The current timestep.
   */
  void ca_exit(void);

  /* clang-format off */
  bool                     m_task_running{false};
  cfsm::collision_tracker m_tracker;
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

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_CRW_HPP_ */
