/**
 * \file oracle_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_ORACLE_ORACLE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ORACLE_ORACLE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace config {
namespace oracle {
struct oracle_manager_config;
} /* namespace oracle */
} /* namespace config */

namespace ds {
class arena_map;
} /* namespace ds */

NS_START(support, oracle);
class entities_oracle;
class tasking_oracle;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class oracle_manager
 * \ingroup fordyca support oracle
 *
 * \brief Thin wrapper around the following oracles:
 *
 * - \ref tasking_oracle
 * - \ref entities_oracle
 */
class oracle_manager {
 public:
  explicit oracle_manager(const config::oracle::oracle_manager_config* config);

  class entities_oracle* entities_oracle(void) { return m_entities.get(); }
  class tasking_oracle* tasking_oracle(void) { return m_tasking.get(); }
  const class entities_oracle* entities_oracle(void) const {
    return m_entities.get();
  }
  const class tasking_oracle* tasking_oracle(void) const {
    return m_tasking.get();
  }

  /**
   * \brief Because the \ref tasking_oracle requires more than just the \ref
   * tasking_oracle_config to construct, it is null-constructed by default, and
   * the loop functions are responsible for actually creating it.
   */
  void tasking_oracle(std::unique_ptr<class tasking_oracle> o);

  /**
   * \brief Update all oracles each timestep (if necessary). Should be called
   * from the loop functions before processing any robots for that timestep (at
   * a minimum).
   */
  void update(ds::arena_map* map);

 private:
  /* clang-format off */
  std::unique_ptr<class entities_oracle> m_entities;
  std::unique_ptr<class tasking_oracle>  m_tasking;
  /* clang-format on */
};

NS_END(oracle, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ORACLE_ORACLE_MANAGER_HPP_ */
