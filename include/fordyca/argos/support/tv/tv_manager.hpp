/**
 * \file tv_manager.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

class env_dynamics;
class fordyca_pd_adaptor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using tv_manager = ctv::tv_manager<env_dynamics, fordyca_pd_adaptor>;

NS_END(tv, support, argos, fordyca);
