/**
 * @file cache_stat_collector.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_CACHE_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_CACHE_STAT_COLLECTOR_HPP_

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

class cache_diagnostics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_stat_collector : public base_stat_collector,
                             public visitor::visitable_any<cache_stat_collector> {
 public:
  explicit cache_stat_collector(const std::string ofname) :
      base_stat_collector(ofname), m_new_data(false), m_stats() {}

  void reset(void) override;
  void collect(const cache_diagnostics& cache);

 private:
  struct stats {
    size_t total_blocks;
    size_t total_pickups;
    size_t total_drops;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  bool m_new_data;
  struct stats m_stats;
};

NS_END(diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_CACHE_STAT_COLLECTOR_HPP_ */
