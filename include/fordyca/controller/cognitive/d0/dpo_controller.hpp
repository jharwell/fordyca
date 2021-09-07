/**
 * \file dpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_DPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_DPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/controller/cognitive/cognitive_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::controller::config::d0 {
 class dpo_controller_repository;
}

NS_START(fordyca);
namespace fsm { namespace d0 { class dpo_fsm; }}

NS_START(controller, cognitive);
class block_sel_matrix;

NS_START(d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_controller
 * \ingroup controller cognitive d0
 *
 * \brief A foraging controller that remembers what it has seen for a period of
 * time (knowledge is modeled by pheromone density and decays as such).
 */
class dpo_controller : public cognitive::cognitive_controller,
                       public rer::client<dpo_controller> {
 public:
  dpo_controller(void) RCPPSW_COLD;
  ~dpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void control_step(void) override;
  void reset(void) override RCPPSW_COLD;
  std::type_index type_index(void) const override { return typeid(*this); }

  /* goal acquisition metrics */
  bool is_phototaxiing_to_goal(bool include_ca) const override RCPPSW_PURE;

  RCPPSW_WRAP_DECL_OVERRIDE(bool, goal_acquired, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_DECL_OVERRIDE(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);

  /**
   * \brief Mutator to allow replacement of the the FSM used by this class to
   * perform foraging tasks (strategy pattern), so that derived classes can
   * reuse the same accessors that this classes provides. Cleaner to do it this
   * way than to have each derived class have its own private version and
   * require duplicate accessors in each derived class.
   */
  void fsm(std::unique_ptr<class fsm::d0::dpo_fsm> fsm);

  fsm::d0::dpo_fsm* fsm(void) { return m_fsm.get(); }
  const fsm::d0::dpo_fsm* fsm(void) const { return m_fsm.get(); }

  const class block_sel_matrix* block_sel_matrix(void) const {
    return m_block_sel_matrix.get();
  }
  /*
   * Needed to update block selections exceptions list in d2.
   */
  class block_sel_matrix* block_sel_matrix(void) {
    return m_block_sel_matrix.get();
  }

 protected:
  /**
   * \brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - DPO perception subsystem (\ref dpo_perception_subsystem)
   * - Block selection matrix (\ref block_sel_matrix)
   */
  void shared_init(const config::d0::dpo_controller_repository& config_repo) RCPPSW_COLD;

 private:
  /**
   * \brief Perform private initialization for the controller:
   *
   * - FSM used to execute foraging. Note that this is NOT included as part of
   *   the shared initialization, because it requires a reference to a
   *   perception subsystem, and derived classes can override the copy
   *   instantiated in \ref shared_init if they wish.
   *
   * - Supervisor FSM.
   *
   * This is called after \ref shared_init() during \ref init().
   */
  void private_init(const config::d0::dpo_controller_repository& config_repo) RCPPSW_COLD;

  /* clang-format off */
  std::unique_ptr<class block_sel_matrix> m_block_sel_matrix;
  std::unique_ptr<fsm::d0::dpo_fsm>       m_fsm;
  /* clang-format on */
};

NS_END(cognitive, d0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D0_DPO_CONTROLLER_HPP_ */
