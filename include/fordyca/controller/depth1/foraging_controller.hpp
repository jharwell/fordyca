/**
 * @file depth1_foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth0/foraging_controller.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "fordyca/tasks/collector.hpp"
#include "fordyca/tasks/forager.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "fordyca/diagnostics/depth1/collectible_task_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

class foraging_sensors;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class foraging_controller : public depth0::foraging_controller,
                            public diagnostics::depth1::collectible_task_diagnostics,
                            public visitor::visitable_any<foraging_controller> {
 public:
  foraging_controller(void) :
      depth0::foraging_controller(),
      m_sensors(),
      m_executive(),
      m_forager(),
      m_collector(),
      m_generalist() {}

  tasks::foraging_task* current_task(void) const;

  /* base diagnostics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 diagnostics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 diagnostics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override;
  std::string task_name(void) const override;

  bool cache_detected(void) const;

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><foraging_controller> section.
   */
  void Init(argos::TConfigurationNode& t_node) override;

  /*
   * @brief Called once every time step; length set in the XML file.
   *
   * Since the FSM does most of the work, this function just tells it run.
   */
  void ControlStep(void) override;

 private:
  std::shared_ptr<foraging_sensors>  m_sensors;
  std::unique_ptr<task_allocation::polled_executive> m_executive;
  std::unique_ptr<tasks::forager> m_forager;
  std::unique_ptr<tasks::collector> m_collector;
  std::unique_ptr<tasks::generalist> m_generalist;
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_CONTROLLER_HPP_ */
