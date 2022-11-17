/**
 * \file registrable.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/creatable_collector_set.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, metrics, registrable);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern rmetrics::creatable_collector_set kStandard;

NS_END(registrable, metrics, ros, fordyca);

