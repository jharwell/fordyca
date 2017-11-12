/**
 * @file block_stat_collector.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_BLOCK_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_BLOCK_STAT_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/diagnostics/base_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics);
namespace visitor = rcppsw::patterns::visitor;

class carryable_object_diagnostics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_stat_collector : public base_stat_collector,
                             public visitor::visitable_any<block_stat_collector> {
 public:
  explicit block_stat_collector(const std::string ofname) :
      base_stat_collector(ofname), m_stats() {}

  void reset(void) override;
  void reset_on_timestep(void) override { m_stats.block_carries = 0; }
  void collect(const carryable_object_diagnostics& block);
  size_t total_collected(void) const { return m_stats.total_collected; }

 private:
  struct stats {
    size_t total_collected; /* aggregate across blocks, not reset each timestep*/
    size_t total_carries; /* aggregate across blocks, not reset each timstep */
    size_t block_carries; /* for one block, reset each timestep */
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats;
};

NS_END(diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_BLOCK_STAT_COLLECTOR_HPP_ */
