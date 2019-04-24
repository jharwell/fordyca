/**
 * @file entities_oracle.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_ORACLE_ENTITIES_ORACLE_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ORACLE_ENTITIES_ORACLE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <boost/optional.hpp>

#include <string>
#include <vector>

#include "fordyca/nsalias.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
namespace oracle {
struct entities_oracle_params;
} /* namespace oracle */
} /* namespace params */
namespace repr {
class base_block;
class base_cache;
} /* namespace repr */
NS_START(support, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class entities_oracle
 * @ingroup fordyca support oracle
 *
 * @brief Repository of perfect knowledge about entities in the arena (blocks,
 * caches, etc). Used to provide an upper bound on the performance of different
 * foraging methods.
 */
class entities_oracle : public rer::client<entities_oracle> {
 public:
  using variant_type = boost::variant<std::shared_ptr<repr::base_block>,
                                      std::shared_ptr<repr::base_cache>>;
  using variant_vector_type = std::vector<variant_type>;

  explicit entities_oracle(const params::oracle::entities_oracle_params* params);

  /**
   * @brief Ask the oracle something.
   *
   * @param query The question to ask. Currently supports:
   *
   * entities.blocks
   * entities.caches
   *
   * @return The answer to the query. Empty answer if query was ill-formed.
   */
  boost::optional<variant_vector_type> ask(const std::string& query) const;

  void set_blocks(const variant_vector_type& ents) { m_blocks = ents; }
  void set_caches(const variant_vector_type& ents) { m_caches = ents; }

  bool caches_enabled(void) const { return mc_caches; }
  bool blocks_enabled(void) const { return mc_blocks; }

 private:
  /* clang-format off */
  const bool          mc_blocks;
  const bool          mc_caches;

  variant_vector_type m_blocks{};
  variant_vector_type m_caches{};
  /* clang-format on */
};

NS_END(oracle, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ORACLE_ENTITIES_ORACLE_HPP_ */
