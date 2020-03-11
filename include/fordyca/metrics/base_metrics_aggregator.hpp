/**
 * \file base_metrics_aggregator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>
#include <typeindex>

#include "rcppsw/er/client.hpp"
#include "rcppsw/metrics/collector_group.hpp"

#include "cosm/metrics/config/metrics_config.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::config {
struct grid_config;
} // namespace cosm::ds::config

NS_START(fordyca);

namespace support {
class base_loop_functions;
}
namespace controller {
class foraging_controller;
} /* namespace controller */

NS_START(metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_metrics_aggregator
 * \ingroup metrics
 *
 * \brief Base class for aggregating collection of metrics for various
 * sources. Extends \ref rmetrics::collector_group to include
 * initialization bits to make loop functions simpler/clearer.
 */
class base_metrics_aggregator : public rer::client<base_metrics_aggregator>,
                                public rmetrics::collector_group {
 public:
  base_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                          const cdconfig::grid_config* const gconfig,
                          const std::string& output_root);
  ~base_metrics_aggregator(void) override = default;

  void collect_from_loop(const support::base_loop_functions* loop);

  /**
   * \brief Collect metrics from a block right before it is dropped in the nest.
   */
  void collect_from_block(const crepr::base_block2D* block);

  /**
   * \brief Collect metrics from \ref foraging_controller.
   */
  void collect_from_controller(const controller::foraging_controller* controller);

  const std::string& metrics_path(void) const { return m_metrics_path; }

 private:
  /* clang-format off */
  std::string m_metrics_path;
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_ */
