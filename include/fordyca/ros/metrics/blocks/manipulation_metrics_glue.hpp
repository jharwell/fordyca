/**
 * \file manipulation_metrics_glue.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
