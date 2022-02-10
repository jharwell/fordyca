/**
 * \file manipulation_metrics_topic_sink.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"

#include "fordyca/ros/metrics/blocks/manipulation_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::metrics::blocks {
class manipulation_metrics_collector;
} /* namespace fordyca::metrics::blocks */

NS_START(fordyca, ros, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class manipulation_metrics_topic_sink
 * \ingroup ros blocks metrics
 *
 * \brief Sink for \ref fmblocks::manipulation_metrics and \ref
 * fmblocks::manipulation_metrics_collector to output metrics to a ROS topic.
 */
class manipulation_metrics_topic_sink final
    : public cros::metrics::topic_sink<frmblocks::manipulation_metrics_msg> {
 public:
  using collector_type = fmblocks::manipulation_metrics_collector;

  manipulation_metrics_topic_sink(const std::string& topic,
                                  const rmetrics::output_mode& mode,
                                  const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(blocks, metrics, ros, fordyca);

