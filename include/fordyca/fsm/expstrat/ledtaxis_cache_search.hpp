/**
 * @file ledtaxis_cache_search.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_CACHE_SEARCH_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_CACHE_SEARCH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "fordyca/fsm/expstrat/ledtaxis.hpp"
#include "fordyca/fsm/expstrat/crw.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ledtaxis_cache_search
 * @ingroup fordyca fsm expstrat
 *
 * @brief Vector to the last known location of a cache, then begin performing
 * CRW at that location, with the idea being that the ledtaxis of another
 * cache being nearby is higher, given that you've found one there before.
 */
class ledtaxis_cache_search : public foraging_expstrat,
                              public rer::client<ledtaxis_cache_search> {
 public:
  explicit ledtaxis_cache_search(const foraging_expstrat::params* const c_params)
      : ledtaxis_cache_search(c_params->saa, c_params->ledtaxis_target) {}
  explicit ledtaxis_cache_search(crfootbot::footbot_saa_subsystem* saa,
                                 const rutils::color& ledtaxis_target)
      : foraging_expstrat(saa),
        ER_CLIENT_INIT("fordyca.fsm.expstrat.ledtaxis_cache_search"),
        m_crw(saa),
        m_taxis(saa, ledtaxis_target) {}

  ~ledtaxis_cache_search(void) override = default;
  ledtaxis_cache_search(const ledtaxis_cache_search&) = delete;
  ledtaxis_cache_search& operator=(const ledtaxis_cache_search&) = delete;

  /* taskable overrides */

  /**
   * @brief Start LED taxis cache search. Crucially, this enables the camera
   * sensor for use during exploration. See #593.
   */
  void task_start(const rta::taskable_argument*) override;

  /**
   * @brief Reset LED taxis cache search after a cache is successfully
   * discovered. Crucially, this disable the camera sensor for increased
   * computational efficiency. See #593.
   */
  void task_reset(void) override final;

  bool task_running(void) const override final {
    return m_taxis.task_running() || m_crw.task_running();
  }

  /**
   * @brief Since we are exploring for something we don't know about, we will
   * never finish (stopping exploration is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;

  /* collision metrics */
  bool in_collision_avoidance(void) const override final RCSW_PURE;
  bool entered_collision_avoidance(void) const override final RCSW_PURE;
  bool exited_collision_avoidance(void) const override final RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override final;
  rmath::vector2u avoidance_loc(void) const override final RCSW_PURE;

  /* prototype overrides */
  std::unique_ptr<foraging_expstrat> clone(void) const override {
    return std::make_unique<ledtaxis_cache_search>(saa(),
                                                   m_taxis.target());
  }

 private:
  /* clang-format off */
  crw      m_crw;
  ledtaxis m_taxis;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_LEDTAXIS_CACHE_SEARCH_HPP_ */
