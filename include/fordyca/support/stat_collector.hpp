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
#include "fordyca/controller/foraging_controller.hpp"

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
    uint total_collected_blocks;
    uint n_resting;
    uint n_exploring;
    uint n_returning;
    uint n_avoiding;
  };

  /* constructors */
  stat_collector(void) : m_stats(), m_ofname(), m_ofile() {}

  /* member functions */
  void reset(const std::string& ofname);
  void finalize(void) { m_ofile.close(); }
  void collect_from_robot(controller::foraging_controller& controller);
  void store(uint timestep);

 private:
  /* member functions */

  /* data members */
  struct foraging_stats m_stats;
  std::string m_ofname;
  std::ofstream m_ofile;
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_STAT_COLLECTOR_HPP_ */
