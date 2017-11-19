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
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth1_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/task_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation { class polled_executive; }}

NS_START(fordyca);
namespace visitor = rcppsw::patterns::visitor;
namespace tasks {
class forager;
class collector;
class generalist;
class foraging_task;
}

NS_START(controller, depth1);
/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class foraging_controller : public depth0::foraging_controller,
                            public metrics::collectible_metrics::robot_metrics::depth1_metrics,
                            public metrics::collectible_metrics::task_metrics,
                            public visitor::visitable_any<foraging_controller> {
 public:
  foraging_controller(void);

  tasks::foraging_task* current_task(void) const;

  /* distance metrics */
  double timestep_distance(void) const override;

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override;
  std::string task_name(void) const override;

  bool cache_detected(void) const;
  bool cache_acquired(void) const;

  void process_los(const representation::line_of_sight* const los) override;

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
  std::unique_ptr<rcppsw::task_allocation::polled_executive> m_executive;
  std::unique_ptr<tasks::forager> m_forager;
  std::unique_ptr<tasks::collector> m_collector;
  std::unique_ptr<tasks::generalist> m_generalist;
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_CONTROLLER_HPP_ */
