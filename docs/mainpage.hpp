/**
 * @mainpage FORDYCA
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
 *
 * @defgroup controller controller
 * Classes defining the different controllers available for use.
 * @{
 *
 * @defgroup depth0 depth0
 * Controllers in which no self-organized dynamic allocation of tasks occurs.
 *
 * @defgroup depth1 depth1
 * Controllers in which self-organized dynamic allocation of tasks DOES occur.
 * @}
 *
 * @defgroup fsm fsm
 * Classes defining the different FSMs used by the controllers and cells in
 * simulation or robot representations of the arena.
 * @{
 *
 * @defgroup depth0 depth0
 * FSMs involving block acquisition and transport to the nest.
 *
 * @defgroup depth1 depth1
 * FSMs involving cache acquisition and transport to a cache.
 * @}
 *
 * @defgroup events events
 * Encapsulation of all events that occur in the arena that robot's and/or the
 * arena loop functions need to react to. Makes heavy use of the visitor design
 * pattern.
 *
 * @defgroup expressions expressions
 * Abstraction/convenience classes for calculating mathematical expressions
 * related to foraging.
 *
 * @defgroup metrics metrics
 * FSM/task metrics definitions and collectors for evaluating swarm performance
 * as the simulation progresses.
 * @{
 *
 * @defgroup fsm fsm
 * Metrics collected from FSMs, and the collectors to collect said metrics.
 * @}
 *
 * @defgroup params params
 * Classes and structs for parsing XML simulation input parameters needed by FORDYCA.
 * @{
 *
 * @defgroup depth0 depth0
 * Classes and structs for parsing XML parameters specific to depth 0 and below.
 *
 * @defgroup depth1 depth1
 * Classes and structs for parsing XML parameters specific to depth 0 and below.
 * @}
 *
 * @defgroup representation representation*
 * Data structures and representation of things in the arena such as blocks,
 * caches, etc.
 *
 * @defgroup support support
 * Classes forming the simulation support framework that enables the robot
 * controllers to function (block/cache distribution/management mainly, some
 * visualization too).
 * @{
 *
 *
 * @defgroup depth1 depth1
 *
 * @}
 *
 * @defgroup tasks tasks
 * Definitions for all high level foraging tasks executable by robots.
 */
