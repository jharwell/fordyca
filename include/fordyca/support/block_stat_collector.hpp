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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_STAT_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/support/base_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
} /* namespace representation */

NS_START(support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_stat_collector : public base_stat_collector {
 public:
  explicit block_stat_collector(const std::string ofname) :
      base_stat_collector(ofname), m_block_stats() {}

  virtual void reset(void);
  void collect(const representation::block& block);
  uint total_collected(void) const { return m_block_stats.total_collected; }

 private:
  struct block_stats {
    uint total_collected;
    uint total_carries;
  };

  std::string csv_header_build(const std::string& header = "");
  std::string csv_line_build(void);

  struct block_stats m_block_stats;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_STAT_COLLECTOR_HPP_ */
