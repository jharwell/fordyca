/**
 * \file bitd_mdpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"

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
 * \class bitd_mdpo_controller
 * \ingroup fordyca controller depth1
 *
 * \brief A controller defining the task allocation space via BIfurcating Task
 * Decomposition (BITD) and spliting the \ref generalist task into the \ref
 * harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 *
 * Uses a Mapped DPO (MDPO) data store for tracking arena state and object
 * relavance.
 *
 * Note that this class does not inherit from \ref depth0::mdpo_controller,
 * because that would mean duplicating all of the executive setup
 * logic/callbacks that are also present in the \ref depth1::bitd_dpo_controller.
 * Cleaner to do it this way.
 */
class bitd_mdpo_controller : public depth1::bitd_dpo_controller,
                           public rer::client<bitd_mdpo_controller> {
 public:
  bitd_mdpo_controller(void) RCSW_COLD;
  ~bitd_mdpo_controller(void) override RCSW_COLD;

  /* base_controller overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void control_step(void) override;
  std::type_index type_index(void) const override { return typeid(*this); }

  mdpo_perception_subsystem* mdpo_perception(void) RCSW_PURE;
  const mdpo_perception_subsystem* mdpo_perception(void) const RCSW_PURE;

 protected:
  /**
   * \brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - Block selection matrix (\ref block_sel_matrix)
   * - Cache selection matrix (\ref cache_sel_matrix)
   * - Task executive (\ref rta::bi_tdgraph_executive)
   * - MDPO perception subsystem (\ref mdpo_perception_subsystem)
   *
   * \param config_repo Handle to parameter repository for this controller
   *                   (after parsing and validation).
   */
  void shared_init(const config::depth1::controller_repository& config_repo) RCSW_COLD;

 private:
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_MDPO_CONTROLLER_HPP_ */
