/**
 * \file manipulation_metrics_msg.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <std_msgs/Header.h>

#include "fordyca/metrics/blocks/manipulation_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct manipulation_metrics_msg : public rmetrics::base_data {
  ::std_msgs::Header                          header{};
  fmetrics::blocks::manipulation_metrics_data data{};
};

NS_END(blocks, metrics, ros, fordyca);
