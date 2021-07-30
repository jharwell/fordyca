/**
 * \file localized_search.hpp
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

#ifndef INCLUDE_FORDYCA_STRATEGY_EXPLORE_LOCALIZED_SEARCH_HPP_
#define INCLUDE_FORDYCA_STRATEGY_EXPLORE_LOCALIZED_SEARCH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/strategy/foraging_strategy.hpp"
#include "rcppsw/math/vector2.hpp"
#include "cosm/spatial/fsm/vector_fsm.hpp"
#include "fordyca/strategy/explore/crw_adaptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class localized_search
 * \ingroup strategy explore
 *
 * \brief An exploration behavior in which robots vector to a specific location,
 * then begin correlated random walk exploration there via \ref
 * crw_adaptor. Falls back to vanilla \ref crw_adaptor if a specific location is
 * not given during at task start.
 */
class localized_search : public foraging_strategy,
                         public rer::client<localized_search> {
 public:
  localized_search(const foraging_strategy::params* const c_params,
                   rmath::rng* rng);

  localized_search(csubsystem::saa_subsystemQ3D* saa,
                   const fsperception::known_objects_accessor* accessor,
                   rmath::rng* rng);

  ~localized_search(void) override = default;
  localized_search(const localized_search&) = delete;
  localized_search& operator=(const localized_search&) = delete;

  /* interference metrics */
  bool exp_interference(void) const override final RCPPSW_PURE;
  bool entered_interference(void) const override final RCPPSW_PURE;
  bool exited_interference(void) const override final RCPPSW_PURE;
  rtypes::timestep interference_duration(void) const override final;
  rmath::vector3z interference_loc3D(void) const override final RCPPSW_PURE;

  /* taskable overrides */

  /**
   * \brief Start the targeted exploration by starting to vector to the starting
   * location.
   *
   * \param c_arg The starting location, or NULL if there is not a
   * location. This can happen if the robot doesn't know of a good starting
   * location (e.g. the location of the last known object of a specific type),
   * in which case we will just fall back to regular CRW.
   */
  void task_start(cta::taskable_argument* c_arg) override {
    if (nullptr != c_arg) {
      m_vfsm.task_start(c_arg);
    }
  }
  void task_reset(void) override final {
    m_vfsm.task_reset();
    m_crw.task_reset();
  }
  bool task_running(void) const override final {
    return m_vfsm.task_running() || m_crw.task_running();
  }

  /**
   * \brief Since we are exploring for something we don't know about, we will
   * never finish (stopping exploration is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;

  /* prototype overrides */
  std::unique_ptr<csstrategy::base_strategy> clone(void) const override {
    return std::make_unique<localized_search>(saa(), accessor(), rng());
  }

 private:
  /* clang-format off */
  csfsm::vector_fsm m_vfsm;
  crw_adaptor      m_crw;
  /* clang-format on */
};

NS_END(explore, strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_EXPLORE_LOCALIZED_SEARCH_HPP_ */
