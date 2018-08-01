/**
 * @file transport_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class transport_metrics_collector
 * @ingroup metrics blocks
 *
 * @brief Collector for \ref transport_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class transport_metrics_collector : public rcppsw::metrics::base_metrics_collector,
                                    public visitor::visitable_any<transport_metrics_collector> {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  transport_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct stats {
    /**
     * @brief  Total # blocks collected in interval.
     */
    uint cum_collected{0};

    /**
     * @brief Total # transporters for collected blocks in interval.
     */
    uint cum_transporters{0};

    /**
     * @brief Total amount of time taken for all collected blocks to be
     * transported from original distribution locations to the nest.
     */
    double cum_transport_time{0.0};

    /**
     * @brief Total amount of time between original arena distribution and first
     * pickup for all collected blocks in interval.
     */
    double cum_initial_wait_time{0.0};
  };

  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats{};
};

NS_END(blocks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_COLLECTOR_HPP_ */
