/**
 * @file base_robot_stat_collector.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_BASE_ROBOT_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_BASE_ROBOT_STAT_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/diagnostics/base_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { class random_foraging_controller; }

NS_START(diagnostics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_robot_stat_collector : public base_stat_collector {
 public:
  explicit base_robot_stat_collector(const std::string ofname) :
      base_stat_collector(ofname), m_base_robot_stats() {}

  void reset() override;
  void collect(const controller::random_foraging_controller& controller);
  void reset_on_timestep(void) override;

 private:
  struct base_robot_stats {
    uint n_exploring_for_block;
    uint n_avoiding_collision;
    uint n_transporting_to_nest;
  };

  std::string csv_header_build(const std::string& header = "") override;
  std::string csv_line_build(void) override;

  struct base_robot_stats m_base_robot_stats;
};

NS_END(diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_BASE_ROBOT_STAT_COLLECTOR_HPP_ */
