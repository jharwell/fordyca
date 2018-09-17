/**
 * @file base_foraging_loop_functions.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/base_foraging_loop_functions.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "fordyca/params/output_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_foraging_loop_functions::base_foraging_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.base") {
}
/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_foraging_loop_functions::output_init(
    const struct params::output_params* const output) {
  if ("__current_date__" == output->output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    m_output_root = output->output_root + "/" +
                    std::to_string(now.date().year()) + "-" +
                    std::to_string(now.date().month()) + "-" +
                    std::to_string(now.date().day()) + ":" +
                    std::to_string(now.time_of_day().hours()) + "-" +
                    std::to_string(now.time_of_day().minutes());
  } else {
    m_output_root = output->output_root + "/" + output->output_dir;
  }

#ifndef ER_NREPORT
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      m_output_root + "/sim.log");
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.events"), m_output_root + "/sim.log");
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.support"), m_output_root + "/sim.log");
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.loop"), m_output_root + "/sim.log");
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.ds.arena_map"),
      m_output_root + "/sim.log");
  client<std::remove_reference<decltype(*this)>::type>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.metrics"),
      m_output_root + "/sim.log");
#endif
} /* output_init() */

void base_foraging_loop_functions::Init(ticpp::Element& node) {
  /* parse all environment parameters and capture in logfile */
  m_params.parse_all(node);

  /* initialize output and metrics collection */
  output_init(m_params.parse_results<params::output_params>());
  m_floor = &GetSpace().GetFloorEntity();
  std::srand(std::time(nullptr));
} /* Init() */

NS_END(support, fordyca);
