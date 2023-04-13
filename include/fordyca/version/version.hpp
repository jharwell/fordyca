/**
 * \file version.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fordyca.hpp"

#include "rcppsw/version/meta_info.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, version);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern rversion::meta_info kVersion;

NS_END(version, fordyca);
