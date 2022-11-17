/**
 * \file registrable.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ros/metrics/registrable.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, metrics, registrable);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
rmetrics::creatable_collector_set kStandard = {
  { typeid(fmetrics::blocks::manipulation_metrics_collector),
    fmspecs::blocks::kManipulation.xml(),
    fmspecs::blocks::kManipulation.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND }
};

NS_END(registrable, metrics, ros, fordyca);
