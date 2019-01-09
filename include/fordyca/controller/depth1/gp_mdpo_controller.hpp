/**
 * @file gp_mdpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GP_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GP_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class mdpo_perception_subsystem;
NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class gp_mdpo_controller
 * @ingroup controller depth1
 *
 * @brief A Greedy Partitioning (GP) controller that switches between \ref generalist,
 * \ref harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks, and uses a Mapped
 * DPO (MDPO) data store for tracking arena state and object relavance.
 *
 * Note that this class does not inherit from \ref depth0::mdpo_controller,
 * because that would mean duplicating all of the executive setup
 * logic/callbacks that are also present in the \ref depth1::gp_dpo_controller.
 * Cleaner to do it this way.
 */
class gp_mdpo_controller : public depth1::gp_dpo_controller,
                           public er::client<gp_mdpo_controller>,
                           public visitor::visitable_any<gp_mdpo_controller>,
                           public metrics::world_model_metrics {
 public:
  gp_mdpo_controller(void);
  ~gp_mdpo_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override;
  double known_percentage(void) const override;
  double unknown_percentage(void) const override;

  const mdpo_perception_subsystem* perception(void) const;
  mdpo_perception_subsystem* perception(void);

 protected:
  /**
   * @brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - Block selection matrix (\ref block_selection_matrix)
   * - Cache selection matrix (\ref cache_selection_matrix)
   * - Task executive (\ref ta::bi_tdgraph_executive)
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   *
   * @param param_repo Handle to parameter repository for this controller (after
   *                   parsing and validation).
   */
  void shared_init(const params::depth1::controller_repository& param_repo);

 private:
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GP_MDPO_CONTROLLER_HPP_ */
