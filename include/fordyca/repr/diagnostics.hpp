/**
 * \file diagnostics.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>

#include "cosm/hal/actuators/diagnostic_actuator.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, repr, diagnostics);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern chactuators::diagnostic_actuator::map_type kColorMap;

NS_END(diagnostics, repr, fordyca);
