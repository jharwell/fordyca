/**
 * @file base_stat_collector.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_STAT_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_STAT_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <fstream>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_stat_collector {
 public:
  explicit base_stat_collector(const std::string ofname) :
      m_ofname(ofname), m_ofile() {}
  virtual ~base_stat_collector(void) {}

  virtual void reset();
  virtual void reset_on_timestep(void) {}
  void csv_line_write(uint timestep);
  void finalize(void) { m_ofile.close(); }

 protected:
  virtual std::string csv_header_build(const std::string& header = "");
  virtual std::string csv_line_build(void) = 0;

  void csv_header_write(void);
  std::ofstream& ofile(void) { return m_ofile; }

 private:
  std::string m_ofname;
  std::ofstream m_ofile;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_STAT_COLLECTOR_HPP_ */
