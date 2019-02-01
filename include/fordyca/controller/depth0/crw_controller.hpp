/**
 * @file crw_controller.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_CRW_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_CRW_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/controller/depth0/depth0_controller.hpp"
#include "rcppsw/patterns/state_machine/base_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw:task_allocation
namespace fsm { namespace depth0 { class crw_fsm; }}

NS_START(controller);
class energy_subsystem;
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;
NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class crw_controller
 * @ingroup controller depth0
 *
 * @brief The most basic form of a foraging controller: roam around randomly
 * until you find a block, and then bring it back to the nest; repeat.
 */
class crw_controller : public depth0_controller,
                       public er::client<crw_controller>,
                       public visitor::visitable_any<crw_controller>,
                       public task_allocation::taskable<crw_controller> {
 public:
  crw_controller(void);
  ~crw_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;
  void Reset(void) override;

  std::type_index type_index(void) const override {
    return std::type_index(typeid(*this));
  }

  /* taskable overrides */
  void task_execute(void) override;
  bool task_finished(void) const override;
  bool task_running(void) const override;
  void task_reset(void) override;
  void task_start(void) override;


  /* goal acquisition metrics */
  bool is_vectoring_to_goal(void) const override { return false; }
  FSM_OVERRIDE_DECL(bool, is_exploring_for_goal, const);
  FSM_OVERRIDE_DECL(bool, goal_acquired, const);
  FSM_OVERRIDE_DECL(acquisition_goal_type, acquisition_goal, const);

  /* block transportation */
  FSM_OVERRIDE_DECL(transport_goal_type, block_transport_goal, const);

  const fsm::depth0::crw_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::depth0::crw_fsm* fsm(void) { return m_fsm.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<fsm::depth0::crw_fsm> m_fsm;
  std::unique_ptr<energy_subsystem> m_energy;
  /* clang-format on */
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH0_CRW_CONTROLLER_HPP_ */
