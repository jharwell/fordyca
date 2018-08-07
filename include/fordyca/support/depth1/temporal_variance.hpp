/**
 * @file cache_penalty_generator.hpp
 *
 * @copyright 2018 John Harwell/Anthony Chen, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_GENERATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_GENERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/depth1/temporal_variance_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

 /*******************************************************************************
  * Classes
  ******************************************************************************/
 /**
  * @class base_temporal_variance
  * @ingroup support depth1
  *
  * @brief Base class for all types of temporal variance to enable strategy
  * pattern.
  */
class base_temporal_variance {
 public:
  explicit base_temporal_variance(
      const params::depth1::temporal_variance_params* const params)
        : m_amplitude(params->amplitude),
          m_period(params->period),
          m_phase_shift(params->phase_shift),
          m_half_period(params->half_period) {}

  /**
   * @brief Get the current value of the variance function, given the current
   * timestep as input.
   */
  virtual uint value(uint timestep) = 0;

  uint amplitude(void) const { return m_amplitude; }
  uint period(void) const { return m_period; }
  uint phase_shift(void) const { return m_phase_shift; }
  uint half_period(void) const { return m_half_period; }

 private:
  uint m_amplitude;
  uint m_period;
  uint m_phase_shift;
  uint m_half_period;
};

class sine_ {
 public:
  sine_(void);



 private:
};

NS_END(depth1, support, fordyca);

#endif  // INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_GENERATOR_HPP_
