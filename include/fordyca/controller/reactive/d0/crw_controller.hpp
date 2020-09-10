/**
 * \file crw_controller.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_REACTIVE_DEPTH0_CRW_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_REACTIVE_DEPTH0_CRW_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca//controller/foraging_controller.hpp"
#include "rcppsw/patterns/fsm/base_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace fsm { namespace d0 { class crw_fsm; }}

NS_START(controller, reactive, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw_controller
 * \ingroup controller reactive d0
 *
 * \brief The most basic form of a foraging controller: roam around randomly
 * until you find a block, and then bring it back to the nest; repeat.
 */
class crw_controller : public foraging_controller,
                       public rer::client<crw_controller> {
 public:
  crw_controller(void) RCSW_COLD;
  ~crw_controller(void) override RCSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void control_step(void) override;
  void reset(void) override RCSW_COLD;
  std::type_index type_index(void) const override { return typeid(*this); }

  /* goal acquisition metrics */
  bool is_vectoring_to_goal(void) const override { return false; }
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const);
  RCPPSW_WRAP_OVERRIDE_DECL(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);

  const fsm::d0::crw_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::d0::crw_fsm* fsm(void) { return m_fsm.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<fsm::d0::crw_fsm> m_fsm;
  /* clang-format on */
};

NS_END(d0, reactive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_REACTIVE_DEPTH0_CRW_CONTROLLER_HPP_ */
