/**
 * \file fordyca.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FORDYCA_HPP_
#define INCLUDE_FORDYCA_FORDYCA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
/**
 * \defgroup fordyca fordyca
 * \brief The root namespace for FORDYCA.
 * @{
 *
 * It is better to define the namespace aliases exported by FORDYCA here, rather
 * than having them be multiply defined in different downstream projects (DRY
 * FTW!).
 *
 * Convention: Namespace aliases from FORDYCA all start with \c 'f', and the
 * first letter of all nested namespaces except the innermost one should be
 * included before the innermost. For example, fordyca::support::d0
 * should have the \c 'f' from \c 'fordyca' and the \c 's' from \c 'support'
 * before the target namespace \c 'd0'.
 *
 * \defgroup config config
 * \brief XML configuration for controllers and simulation manager.
 *
 * @{
 *
 * \defgroup arena arena
 * \brief XML configuration and parsing for the \ref arena_map, block
 * distribution classes.
 *
 * \defgroup block_sel block_sel
 * \brief XML configuration and parsing for the \ref block_sel_matrix and
 * components.
 *
 * \defgroup cache_sel cache_sel
 * \brief XML configuration and parsing for the \ref cache_sel_matrix and
 * components.
 *
 * \defgroup caches caches
 * \brief XML configuration and parsing for static/dynamic caches in the arena.
 *
 * \defgroup oracle oracle
 * \brief XML configuration and parsing for the \ref oracle_manager and its
 * components.
 *
 * \defgroup perception perception
 * \brief XML configuration and parsing for the perception subsystems used by
 * some controllers.
 *
 * \defgroup tv tv
 * \brief XML configuration and parsing for the \ref tv_manager and its
 * components.
 *
 * \defgroup d0 d0
 * \brief XML configuration and parsing for d0 controllers (those which do
 * not employ partitioning/use caches).
 *
 * \defgroup d1 d1
 * \brief XML configuration and parsing for d1 controllers (those which
 * utilize a single static cache).
 *
 * \defgroup d1 d1
 * \brief XML configuration and parsing for d2 controllers (those which
 * dynamically create, utilize, and deplete caches).
 *
 * @}
 *
 * \defgroup controller controller
 * \brief Robot controllers, including common components used across controllers
 * of varying complexities.
 *
 * @{
 *
 * \defgroup d0 d0
 * \brief Depth0 controllers (those which do not employ partitioning/use
 * caches).
 *
 * \defgroup d1 d1
 * \brief Depth1 controllers (those which utilize a single static cache).
 *
 * \defgroup d2 d2
 * \brief Depth2 controllers (those which dynamically create, utilize, and
 * deplete caches).
 *
 * @}
 *
 * \defgroup ds ds
 * \brief Data structures.
 *
 * \defgroup events events
 * \brief Simulation events which allow the robots to interact with the arena.
 *
 * \defgroup fsm fsm
 * \brief Foraging FSMs, the implementation mechanism for the foraging tasks.
 *
 * \defgroup math math
 * \brief Mathematical calculations for various utilities.
 *
 * \defgroup metrics
 * \brief Metric collection interfaces and collectors.
 *
 * @{
 *
 * \defgroup blocks blocks
 * \brief Metrics collected from/relating to blocks in the arena.
 *
 * \defgroup caches caches
 * \brief Metrics collected from/relating to caches in the arena.
 *
 * \defgroup Perception perception
 * \brief Metrics collected from/relating to the the collective perceived state
 * of the arena by robots in the swarm.
 *
 * @}
 *
 * \defgroup repr repr
 * \brief Representations of entities within the arena.
 *
 * \defgroup support support
 * \brief Foraging support (i.e. the things that are needed to support swarm
 * foraging but that are not part of robot controllers).
 *
 * @{
 *
 * \defgroup d0 d0
 * \brief Support classes specific for d0 controllers (those which do not
 * employ partitioning/use caches).
 *
 * \defgroup d1 d1
 * \brief Support classes specific for d1 controllers (those which utilize a
 * single static cache).
 *
 * \defgroup d2 d2
 * \brief Support classes specific for d2 controllers (those which dynamic
 * create, utilize, and deplete caches).
 *
 * @}
 *
 * \defgroup tasks tasks
 * \brief Definition of top level foraging tasks available to robots for
 * execution.
 */

namespace fordyca {
namespace config {
namespace strategy {}
} /* namespace config */

namespace strategy {
namespace explore {}
} /* namespace strategy */

namespace controller {
namespace cognitive {}
namespace reactive {}
}

namespace ds {}

} /* namespace fordyca */

namespace fconfig = fordyca::config;
namespace fcstrategy = fconfig::strategy;

namespace fds = fordyca::ds;

namespace fstrategy = fordyca::strategy;
namespace fsexplore = fstrategy::explore;

namespace fcontroller = fordyca::controller;
namespace fccognitive = fcontroller::cognitive;
namespace fcreactive = fcontroller::reactive;

#endif /* INCLUDE_FORDYCA_FORDYCA_HPP_ */
