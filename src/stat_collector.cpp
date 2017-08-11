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
void stat_collector::reset(const std::string& ofname) {
  m_ofname = ofname;
  /* Open output file and truncate */
  if (m_ofile.is_open()) {
    m_ofile.close();
  }
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_ofile << "clock\tcollected_blocks\tresting\texploring\treturning\tcollision_avoidance\n";
} /* reset() */

void stat_collector::collect_from_robot(controller::foraging_controller& controller) {
  /* Count how many foot-bots are in which state */
  m_stats.n_resting += controller.is_resting();
  m_stats.n_exploring += controller.is_exploring();
  m_stats.n_returning += controller.is_returning();
  m_stats.n_avoiding += controller.is_avoiding_collision();

  if (controller.is_carrying_block() && controller.in_nest()) {
    ++m_stats.total_collected_blocks;
  }
} /* collect_from_robot() */

void stat_collector::store(uint timestep) {
  /* Output stuff to file */
  m_ofile << timestep << "\t"
          << m_stats.total_collected_blocks << "\t"
          << m_stats.n_resting << "\t"
          << m_stats.n_exploring << "\t"
          << m_stats.n_returning << "\t"
          << m_stats.n_avoiding << std::endl;

} /* store() */


NS_END(support, fordyca);
