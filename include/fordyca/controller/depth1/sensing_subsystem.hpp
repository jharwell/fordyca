/**
 * @file depth1/sensing_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_SENSING_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_SENSING_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth0/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class sensing_subsystem
 * @ingroup controller depth1
 *
 * @brief The sensors used by the depth1 foraging controller.
 */
class sensing_subsystem: public depth0::sensing_subsystem {
 public:
  sensing_subsystem(
      const struct params::sensing_params* c_params,
      const struct base_sensing_subsystem::sensor_list* list);

  sensing_subsystem(const sensing_subsystem& fsm) = delete;
  sensing_subsystem& operator=(const sensing_subsystem& fsm) = delete;

  /**
   * @brief If \c TRUE, a cache has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool cache_detected(void);
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_SENSING_SUBSYSTEM_HPP_ */
