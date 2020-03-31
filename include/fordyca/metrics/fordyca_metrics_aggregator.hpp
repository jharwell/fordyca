/**
 * \file fordyca_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/metrics/base_metrics_aggregator.hpp"
#include "cosm/metrics/config/metrics_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
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
 * \class fordyca_metrics_aggregator
 * \ingroup metrics
 *
 * \brief Extends the \ref cmetrics::base_metrics_aggregator for the FORDYCA
 * project.
 */
class fordyca_metrics_aggregator
    : public rer::client<fordyca_metrics_aggregator>,
      public cmetrics::base_metrics_aggregator {
 public:
  fordyca_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                             const cdconfig::grid_config* const gconfig,
                             const std::string& output_root);
  ~fordyca_metrics_aggregator(void) override = default;

  void collect_from_loop(const support::base_loop_functions* loop);
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_AGGREGATOR_HPP_ */
