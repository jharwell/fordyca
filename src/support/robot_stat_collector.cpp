/**
 * @file robot_stat_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/robot_stat_collector.hpp"
#include "fordyca/controller/random_foraging_controller.hpp"
#include "fordyca/controller/memory_foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string robot_stat_collector::csv_header_build(const std::string& header) {
  return base_stat_collector::csv_header_build(header) +
      "n_searching;n_exploring;n_returning;n_collision_avoidance;n_vectoring";
} /* csv_header_build() */

void robot_stat_collector::reset(void) {
  base_stat_collector::reset();
  reset_on_timestep();
} /* reset() */

void robot_stat_collector::collect(
    const controller::random_foraging_controller& controller) {
  m_robot_stats.n_exploring += controller.is_exploring();
  m_robot_stats.n_returning += controller.is_returning();
  m_robot_stats.n_avoiding += controller.is_avoiding_collision();
} /* collect() */

void robot_stat_collector::collect(
    const controller::memory_foraging_controller& controller) {
  m_robot_stats.n_searching += controller.is_searching_for_block();
  m_robot_stats.n_exploring += controller.is_exploring();
  m_robot_stats.n_returning += controller.is_returning();
  m_robot_stats.n_avoiding += controller.is_avoiding_collision();
  m_robot_stats.n_vectoring += controller.is_vectoring();
} /* collect() */

std::string robot_stat_collector::csv_line_build(void) {
  return std::to_string(m_robot_stats.n_searching) + ";" +
      std::to_string(m_robot_stats.n_exploring) + ";" +
      std::to_string(m_robot_stats.n_returning) + ";" +
      std::to_string(m_robot_stats.n_avoiding) + ";" +
      std::to_string(m_robot_stats.n_vectoring) + ";";
} /* store_foraging_stats() */

void robot_stat_collector::reset_on_timestep(void) {
  m_robot_stats = {0, 0, 0, 0, 0};
} /* reset_on_timestep() */


NS_END(support, fordyca);
