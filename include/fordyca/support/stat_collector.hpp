/**
 * @file stat_collector.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_STAT_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class stat_collector {
 public:
  struct foraging_stats {
    uint n_exploring;
    uint n_returning;
    uint n_avoiding;
  };

  struct block_stats {
    uint total_collected;
    uint total_carries;
  };

  explicit stat_collector(const std::string ofname) :
      m_foraging_stats(), m_block_stats(), m_ofname(ofname), m_ofile() {}

  void reset();
  void finalize(void) { m_ofile.close(); }
  uint n_collected_blocks(void) const { return m_block_stats.total_collected; }

  /**
   * @brief Collect statistics from a robot at the end of a timestep. Must be
   * called before the robot state is updated (i.e. pickup/dropped blocks) by an
   * external functions.
   *
   * @param controller The controller to collect from.
   */
  void collect_from_robot(const controller::base_controller& controller);
  void collect_from_block(const representation::block& block);
  void store_foraging_stats(uint timestep);
  void store_block_stats(uint timestep);

 private:
  /* member functions */

  /* data members */
  struct foraging_stats m_foraging_stats;
  struct block_stats m_block_stats;
  std::string m_ofname;
  std::ofstream m_ofile;
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_STAT_COLLECTOR_HPP_ */
