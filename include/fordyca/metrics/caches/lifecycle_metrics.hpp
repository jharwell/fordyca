/**
 * \file lifecycle_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/types/timestep.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lifecycle_metrics
 * \ingroup metrics caches
 *
 * \brief Interface defining lifecycle metrics that can be collected on
 * static/dynamic caches in the arena during their lifetime.
 */
class lifecycle_metrics : public rmetrics::base_metrics {
 public:
  lifecycle_metrics(void) = default;
  ~lifecycle_metrics(void) override = default;
  lifecycle_metrics(const lifecycle_metrics&) = default;
  lifecycle_metrics& operator=(const lifecycle_metrics&) = default;

  /**
   * \brief Should return the # caches that have been created in the arena this
   * timestep.
   */
  virtual size_t caches_created(void) const = 0;

  /**
   * \brief Should return the # caches that have been created, but then
   * discarded due to constraint violation in the arena this timestep.
   */
  virtual size_t caches_discarded(void) const = 0;

  /**
   * \brief Should return the # of caches that have been depleted in the arena
   * this timestep.
   */
  virtual size_t caches_depleted(void) const = 0;

  /**
   * \brief Should return the ages of the caches that were depleted this
   * timestep (i.e. how many timesteps did they exist before being depleted?).
   */
  virtual std::vector<rtypes::timestep> cache_depletion_ages(void) const = 0;
};

NS_END(caches, metrics, fordyca);

