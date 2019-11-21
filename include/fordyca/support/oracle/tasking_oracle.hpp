/**
 * \file tasking_oracle.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_ORACLE_TASKING_ORACLE_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ORACLE_TASKING_ORACLE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <boost/optional.hpp>

#include <map>
#include <string>

#include "fordyca/fordyca.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw::ta {
class bi_tdgraph_executive;
class polled_task;
namespace ds {
class bi_tdgraph;
} /* namespace ds */
} // namespace rcppsw::ta

NS_START(fordyca);
namespace config {
namespace oracle {
struct tasking_oracle_config;
} /* namespace oracle */
} /* namespace config */
NS_START(support, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tasking_oracle
 * \ingroup fordyca support oracle
 *
 * \brief Repository of perfect knowledge about swarm level task
 * allocation. Used to provide an upper bound on the performance of different
 * allocation methods.
 */
class tasking_oracle final : public rer::client<tasking_oracle> {
 public:
  using variant_type = boost::variant<rta::time_estimate>;

  tasking_oracle(const config::oracle::tasking_oracle_config* config,
                 const rta::ds::bi_tdgraph* graph);

  /**
   * \brief Ask the oracle something.
   *
   * \param query The question to ask. Currently supports:
   *
   * exec_est.\<task name\>
   * interface_est.\<task name\>
   *
   * \return The answer to the query. Empty answer if query was ill-formed.
   */
  boost::optional<variant_type> ask(const std::string& query) const;

  /**
   * \brief Adds the oracle to the task finish and task abort callback lists for
   * the specified executive. Should be called once during initialization to
   * attach the oracle to each robot so that it can build a perfect map of task
   * allocation information as the simulation progresses.
   *
   * This results in asynchronous/irregular updates to the oracle's map of task
   * allocation information as robots finish/abort tasks.
   */
  void listener_add(rta::bi_tdgraph_executive* executive);

  bool update_exec_ests(void) const { return mc_exec_ests; }
  bool update_int_ests(void) const { return mc_int_ests; }

  void task_abort_cb(const rta::polled_task* task);
  void task_finish_cb(const rta::polled_task* task);

 private:
  /* clang-format off */
  const bool                         mc_exec_ests;
  const bool                         mc_int_ests;
  std::map<std::string, variant_type> m_map{};
  /* clang-format on */
};

NS_END(oracle, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ORACLE_TASKING_ORACLE_HPP_ */
