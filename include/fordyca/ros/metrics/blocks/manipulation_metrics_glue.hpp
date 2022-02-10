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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "cosm/pal/pal.hpp"

#include "fordyca/ros/metrics/blocks/manipulation_metrics_msg.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/

NS_START(cosm, ros, metrics, msg_traits);

template<>
struct payload_type<frmblocks::manipulation_metrics_msg> {
  using type = fmblocks::manipulation_metrics_data;
};

NS_END(msg_traits, metrics, ros, cosm);


/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<frmblocks::manipulation_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const frmblocks::manipulation_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<frmblocks::manipulation_metrics_msg> {
  static const char* value() {
    return "fordyca_msgs/manipulation_metrics_data";
  }
  static const char* value(const frmblocks::manipulation_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<frmblocks::manipulation_metrics_msg> {
  static const char* value() {
    return "See FORDYCA docs for documentation.";
  }
  static const char* value(const frmblocks::manipulation_metrics_msg&) {
    return value();
  }
};

template <>
struct HasHeader<frmblocks::manipulation_metrics_msg> : TrueType {};

NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<frmblocks::manipulation_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    for (size_t i = 0; i < fmblocks::block_manip_events::ekMAX_EVENTS; ++i) {
      stream.next(t.data.interval[i].events);
      stream.next(t.data.interval[i].penalties);

      stream.next(t.data.cum[i].events);
      stream.next(t.data.cum[i].penalties);
    } /* for(i..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);
