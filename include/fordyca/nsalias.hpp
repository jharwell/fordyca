/**
 * @file nsalias.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_NSALIAS_HPP_
#define INCLUDE_FORDYCA_NSALIAS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rcppsw {
namespace metrics {
} /* namespace metrics */
namespace patterns {
namespace state_machine {
} /* namespace state_machine */
namespace visitor {
} /* namespace visitor */
namespace factory {
} /* namespace factory */
namespace prototype {
} /* namespace prototype */
namespace decorator {
} /* namespace decorator */
} /* namespace patterns */
namespace swarm {
namespace convergence {
} /* namespace convergence */
} /* namespace swarm */
namespace math {
} /* namespace math */
namespace ta {
} /* namespace ta */
namespace algorithm {
} /* namespace algorithm */
namespace utils {
} /* namespace utils */
namespace er {
} /* namespace er */
namespace control {
} /* namespace control */
namespace config {
} /* namespace config */
namespace ds {
} /* namespace ds */
namespace mpl {
} /* namespace mpl */
namespace robotics {
namespace kin2D {
} /* namespace kin2D */
namespace steer2D {
} /* namespace steer2D */
namespace hal {
} /* namespace hal */
} /* namespace robotics */
namespace types {
} /* namespace types */
} /* namespace rcppsw */

NS_START(fordyca);

/*
 * Convention: Namespace aliases from rcppsw all start with 'r', and the first
 * letter of all nested namespaces except the innermost one should be included
 * before the innermost. For example, rcppsw::patterns::visitor should have the
 * 'r' from 'rcppsw' and the 'p' from 'patterns' before the target namespace
 * 'visitor'.
 */
namespace rmetrics = rcppsw::metrics;
namespace rpfsm = rcppsw::patterns::state_machine;
namespace rpvisitor = rcppsw::patterns::visitor;
namespace rpfactory = rcppsw::patterns::factory;
namespace rpprototype = rcppsw::patterns::prototype;
namespace rpdecorator = rcppsw::patterns::decorator;
namespace rswarm = rcppsw::swarm;
namespace rmath = rcppsw::math;
namespace rta = rcppsw::ta;
namespace ralg = rcppsw::algorithm;
namespace rutils = rcppsw::utils;
namespace rer = rcppsw::er;
namespace rct = rcppsw::control;
namespace rconfig = rcppsw::config;
namespace rds = rcppsw::ds;
namespace rmpl = rcppsw::mpl;
namespace rrobotics = rcppsw::robotics;
namespace rrhal = rrobotics::hal;
namespace rrkin2D = rrobotics::kin2D;
namespace rrsteer2D = rrobotics::steer2D;
namespace rsc = rswarm::convergence;
namespace rtypes = rcppsw::types;

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_NSALIAS_HPP_ */
