/**
 * @file cache_update_handler.hpp
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
 */

#ifndef INCLUDE_FORDYCA_SUPPORT_CACHE_UPDATE_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CACHE_UPDATE_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>

#include "rcppsw/common/er_client.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/representation/real_coord.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_update_handler : public rcppsw::common::er_client,
                             public visitor::visitable<cache_update_handler> {
 public:
  cache_update_handler(std::shared_ptr<rcppsw::common::er_server> server,
                       std::shared_ptr<std::vector<representation::cache>> caches);

  representation::cache* map_to_cache(const representation::block* const block);
  void block_add(representation::cache* cache,
                 representation::block* const block);
  void block_remove(representation::cache* cache,
                    representation::block* const block);

 private:
  std::shared_ptr<std::vector<representation::cache>> m_caches;
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHE_UPDATE_HANDLER_HPP_ */
