/**
 * @file energy_opt_metrics.hpp
 *
 * @copyright 2018 Anthony Chen/John Harwell, All rights reserved.
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

 #ifndef INCLUDE_METRICS_energy_opt_metrics_HPP_
 #define INCLUDE_METRICS_energy_opt_metrics_HPP_

 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include "rcppsw/metrics/base_metrics.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, metrics, energy);

 /*******************************************************************************
  * Class Definitions
  ******************************************************************************/

 /**
  * @class energy_opt_metrics
  * @ingroup metrics energy
  *
  * @brief Defines the metrics to be collected from robots about the energy and
  * motion.
  *
  *
  * Metrics are collected every timestep
  */
  class energy_metrics : virtual public rcppsw::metrics::base_metrics {
   public:
    energy_metrics(void) = default;

    /**
     * @brief Return the average energy level
     */
    virtual double energy_level(void) const = 0;

    /**
     * @brief Return the number of robots in the nest
     */
    virtual int is_charging(void) const = 0;

  };

  NS_END(energy, metrics, fordyca);

  #endif /* INCLUDE_FORDYCA_METRICS_ENERGY_OPT_METRICS_HPP_ */
