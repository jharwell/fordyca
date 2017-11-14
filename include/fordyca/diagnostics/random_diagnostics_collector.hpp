/**
 * @file random_diagnostics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_RANDOM_DIAGNOSTICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_RANDOM_DIAGNOSTICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/diagnostics/base_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics);

class random_collectible_diagnostics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class random_diagnostics_collector : public base_stat_collector {
 public:
  explicit random_diagnostics_collector(const std::string ofname) :
      base_stat_collector(ofname), m_stats() {}

  void reset() override;
  void collect(const random_collectible_diagnostics& diag);
  void reset_on_timestep(void) override;

 private:
  struct sim_stats {
    size_t n_exploring_for_block;
    size_t n_avoiding_collision;
    size_t n_transporting_to_nest;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  struct sim_stats m_stats;
};

NS_END(diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_RANDOM_DIAGNOSTICS_COLLECTOR_HPP_ */
