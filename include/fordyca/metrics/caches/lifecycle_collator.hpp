/**
 * @file lifecycle_collator.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_COLLATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_COLLATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class lifecycle_collator
 * @ingroup metrics caches
 *
 * @brief Collates information about the lifecycles of caches in the arena for
 * the purpose of metric collection.
 */
class lifecycle_collator : public lifecycle_metrics {
 public:
  lifecycle_collator(void) = default;

  uint caches_created(void) const override { return m_created; }
  uint caches_depleted(void) const override { return m_depleted; }
  void caches_created(uint c) { m_created += c; }
  void caches_depleted(uint c) { m_depleted += c; }
  void reset_metrics(void) override { m_created = 0; m_depleted = 0;  }

  void cache_created(void) { ++m_created; }
  void cache_depleted(void) { ++m_depleted; }

 private:
  uint m_created{0};
  uint m_depleted{0};
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_COLLATOR_HPP_ */
