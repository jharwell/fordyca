/**
 * \file manipulation_metrics_glue.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_GLUE_HPP_
#define INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_GLUE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

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
      stream.next(t.interval[i].events);
      stream.next(t.interval[i].penalties);
    } /* for(i..) */

    for (size_t i = 0; i < fmblocks::block_manip_events::ekMAX_EVENTS; ++i) {
      stream.next(t.cum[i].events);
      stream.next(t.cum[i].penalties);
    } /* for(i..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);

#endif /* INCLUDE_FORDYCA_ROS_METRICS_BLOCKS_MANIPULATION_METRICS_GLUE_HPP_ */
