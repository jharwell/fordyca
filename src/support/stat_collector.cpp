/**
 * @file stat_collector.cpp
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
#include "fordyca/support/stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stat_collector::reset(void) {
  /* Open output file and truncate */
  if (m_ofile.is_open()) {
    m_ofile.close();
  }
  m_block_stats = {0, 0};
  m_foraging_stats = {0, 0, 0};
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_ofile << "clock\tcollected_blocks\tavg_carries\texploring\treturning\tcollision_avoidance\n";
} /* reset() */

void stat_collector::collect_from_robot(
    const controller::base_controller& controller) {
  /* Count how many foot-bots are in which state */
  m_foraging_stats.n_exploring += controller.is_exploring();
  m_foraging_stats.n_returning += controller.is_returning();
  m_foraging_stats.n_avoiding += controller.is_avoiding_collision();
} /* collect_from_robot() */

void stat_collector::collect_from_block(const representation::block& block) {
  ++m_block_stats.total_collected;
  m_block_stats.total_carries += block.carries();
} /* collect_from_block() */

void stat_collector::store_foraging_stats(uint timestep) {
  /* Output stuff to file */
  double avg_carries = 0;
  if (m_block_stats.total_collected > 0) {
    avg_carries = (double)m_block_stats.total_collected/m_block_stats.total_carries;
  }
  m_ofile << timestep << "\t"
          << m_block_stats.total_collected << "\t"
          << avg_carries << "\t"
          << m_foraging_stats.n_exploring << "\t"
          << m_foraging_stats.n_returning << "\t"
          << m_foraging_stats.n_avoiding << std::endl;
  m_foraging_stats.n_exploring = 0;
  m_foraging_stats.n_returning = 0;
  m_foraging_stats.n_avoiding = 0;
} /* store_foraging_stats() */


NS_END(support, fordyca);
