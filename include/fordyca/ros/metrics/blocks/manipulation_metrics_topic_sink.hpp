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

#ifndef INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_data.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<fmblocks::manipulation_metrics_data> {
  static const char* value() {
    return MD5Sum<fmblocks::manipulation_metrics_data>::value();
  }
  static const char* value(const fmblocks::manipulation_metrics_data& m) {
    return MD5Sum<fmblocks::manipulation_metrics_data>::value(m);
  }
};
template <>
struct DataType<fmblocks::manipulation_metrics_data> {
  static const char* value() {
    return DataType<fmblocks::manipulation_metrics_data>::value();
  }
  static const char* value(const fmblocks::manipulation_metrics_data& m) {
    return DataType<fmblocks::manipulation_metrics_data>::value(m);
  }
};

template<>
struct Definition<fmblocks::manipulation_metrics_data> {
  static const char* value() {
    return Definition<fmblocks::manipulation_metrics_data>::value();
  }
  static const char* value(const fmblocks::manipulation_metrics_data& m) {
    return Definition<fmblocks::manipulation_metrics_data>::value(m);
  }
};
NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<fmblocks::manipulation_metrics_data> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    for (size_t i = 0; i < fmblocks::block_manip_events::ekMAX_EVENTS; ++i) {
      stream.next(t.interval[i].events.load());
      stream.next(t.interval[i].penalties.load());
    } /* for(i..) */

    for (size_t i = 0; i < fmblocks::block_manip_events::ekMAX_EVENTS; ++i) {
      stream.next(t.cum[i].events.load());
      stream.next(t.cum[i].penalties.load());
    } /* for(i..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);

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
    : public cros::metrics::topic_sink<fmblocks::manipulation_metrics_data> {
 public:
  using collector_type = fmblocks::manipulation_metrics_collector;

  manipulation_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(blocks, metrics, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_TOPIC_SINK_HPP_ */
