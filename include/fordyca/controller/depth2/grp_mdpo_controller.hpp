/**
 * @file grp_mdpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class mdpo_perception_subsystem;
NS_START(depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class grp_mdpo_controller
 * @ingroup fordyca controller depth2
 *
 * @brief A Greedy Recursive Partitioning controller that moves through a depth2
 * recursive task decomposition graph, changing task according to dynamic
 * changes in the environment and/or execution/interface times of the tasks, and
 * using a Mapped DPO data store for tracking arena state and object relevance.
 */
class grp_mdpo_controller : public depth2::grp_dpo_controller,
                            public rer::client<grp_mdpo_controller> {
 public:
  grp_mdpo_controller(void);
  ~grp_mdpo_controller(void) override = default;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;

  void shared_init(const config::depth2::controller_repository& config_repo);

  mdpo_perception_subsystem* mdpo_perception(void);
  const mdpo_perception_subsystem* mdpo_perception(void) const;
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_ */
