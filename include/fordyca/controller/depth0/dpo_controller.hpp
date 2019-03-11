/**
 * @file dpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_DPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_DPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm { namespace depth0 { class dpo_fsm; }}

namespace params {
namespace depth0 { class dpo_controller_repository; }
}

NS_START(controller);
class base_perception_subsystem;
class dpo_perception_subsystem;
class block_sel_matrix;
namespace depth0 { class sensing_subsystem; }

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_controller
 * @ingroup controller depth0
 *
 * @brief A foraging controller that remembers what it has seen for a period of
 * time (knowledge is modeled by pheromone density and decays as such).
 */
class dpo_controller : public crw_controller,
                       public er::client<dpo_controller>,
                       public visitor::visitable_any<dpo_controller> {
 public:
  dpo_controller(void);
  ~dpo_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;
  void Reset(void) override;

  std::type_index type_index(void) const override {
    return std::type_index(typeid(*this));
  }

  /* goal acquisition metrics */
  FSM_OVERRIDE_DECL(bool, goal_acquired, const);
  FSM_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  FSM_OVERRIDE_DECL(bool, is_exploring_for_goal, const);
  FSM_OVERRIDE_DECL(acquisition_goal_type, acquisition_goal, const);

  /* block transportation */
  FSM_OVERRIDE_DECL(transport_goal_type, block_transport_goal, const);

  /**
   * @brief Set the robot's current line of sight (LOS).
   */
  void los(std::unique_ptr<repr::line_of_sight> new_los);
  double los_dim(void) const;

  /**
   * @brief Get the current LOS for the robot.
   */
  const repr::line_of_sight* los(void) const;

  /**
   * @brief Set whether or not a robot is supposed to display it's LOS as a
   * square of the appropriate size during simulation.
   */
  void display_los(bool display_los) { m_display_los = display_los; }

  /**
   * @brief If \c TRUE, then the robot should display its approximate LOS as a
   * circle on the ground during simulation.
   */
  bool display_los(void) const { return m_display_los; }

  const base_perception_subsystem* perception(void) const {
    return m_perception.get();
  }
  base_perception_subsystem* perception(void) { return m_perception.get(); }

  dpo_perception_subsystem* dpo_perception(void);
  const dpo_perception_subsystem* dpo_perception(void) const {
    return const_cast<dpo_controller*>(this)->dpo_perception();
  }

  const fsm::depth0::dpo_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::depth0::dpo_fsm* fsm(void) { return m_fsm.get(); }

  const class block_sel_matrix* block_sel_matrix(void) const {
    return m_block_sel_matrix.get();
  }
  /*
   * Needed to update block selections exceptions list in depth2.
   */
  class block_sel_matrix* block_sel_matrix(void) {
    return m_block_sel_matrix.get();
  }

 protected:
  /**
   * @brief Mutator to allow replacement of the perception subsystem object
   * managed by the the controller (strategy pattern), so that derived classes
   * can reuse the same accessors that this classes provides. Cleaner to do it
   * this way than to have each derived class have its own private version and
   * require duplicate accessors in each derived class.
   */
  void perception(std::unique_ptr<base_perception_subsystem> perception);

  /**
   * @brief Mutator to allow replacement of the the FSM used by this class to
   * perform foraging tasks (strategy pattern), so that derived classes can
   * reuse the same accessors that this classes provides. Cleaner to do it this
   * way than to have each derived class have its own private version and
   * require duplicate accessors in each derived class.
   */
  void fsm(std::unique_ptr<class fsm::depth0::dpo_fsm> fsm);

  /**
   * @brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - DPO perception subsystem (\ref dpo_perception_subsystem)
   * - Block selection matrix (\ref block_sel_matrix)
   */
  void shared_init(const params::depth0::dpo_controller_repository& param_repo);

 private:
  /**
   * @brief Perform private initialization for the controller:
   *
   * - FSM used to execute foraging. Note that this is NOT included as part of
   *   the shared initialization, because it requires a reference to a
   *   perception subsystem, and derived classes can override the copy
   *   instantiated in \ref shared_init if they wish.
   *
   * This is called after \ref shared_init() during \ref Init().xo
   */
  void private_init(void);

  /* clang-format off */
  bool                                       m_display_los{false};
  rmath::vector2d                            m_light_loc;
  std::unique_ptr<class block_sel_matrix>    m_block_sel_matrix;
  std::unique_ptr<base_perception_subsystem> m_perception;
  std::unique_ptr<fsm::depth0::dpo_fsm>      m_fsm;
  /* clang-format on */
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DPO_CONTROLLER_HPP_ */
