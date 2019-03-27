/**
 * @file mdpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { namespace depth0 { class mdpo_controller_repository; }}

NS_START(controller);

class mdpo_perception_subsystem;

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class mdpo_controller
 * @ingroup controller depth0
 *
 * @brief A foraging controller that remembers what it has seen for a period of
 * time (knowledge is modeled by pheromone density and decays as such). It
 * models the state of the environment (empty, unknown, contains object, etc.)
 * AND the objects within the environment.
 *
 * It shares the underlying FSM with the \ref dpo_controller so that the metrics
 * collection functions can be reused.
 */
class mdpo_controller : public dpo_controller,
                        public er::client<mdpo_controller>,
                        public metrics::world_model_metrics {
 public:
  mdpo_controller(void);
  ~mdpo_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  std::type_index type_index(void) const override {
    return std::type_index(typeid(*this));
  }

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override;
  double known_percentage(void) const override;
  double unknown_percentage(void) const override;

  mdpo_perception_subsystem* mdpo_perception(void);
  const mdpo_perception_subsystem* mdpo_perception(void) const {
    return const_cast<mdpo_controller*>(this)->mdpo_perception();
  }

  /**
   * @brief Initialization that derived classes may also need to perform, if the
   * want to use any of the following parts of this class's functionality as-is:
   *
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   * - Block selection matrix (\ref block_sel_matrix)
   */
  void shared_init(const params::depth0::mdpo_controller_repository& param_repo);

 private:
  /**
   * @brief Perform initialization private to this class:
   *
   * - DPO FSM
   *
   * Called after \ref shared_init() in \ref Init().
   */
  void private_init(void);
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_MDPO_CONTROLLER_HPP_ */
