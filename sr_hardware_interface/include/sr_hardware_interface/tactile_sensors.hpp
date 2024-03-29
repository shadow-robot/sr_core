/*
* Copyright 2011, 2023 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file   tactile_sensors.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Oct  5 14:57:27 2011
 * @brief  Contains the different tactile sensors structures.
 *
 *
 */

#ifndef _TACTILE_SENSORS_HPP_
#define _TACTILE_SENSORS_HPP_

#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <boost/circular_buffer.hpp>
#include <geometry_msgs/Point.h>
#include <sstream>

#include <ros/ros.h>

namespace tactiles
{
class GenericTactileData
{
public:
  GenericTactileData()
  {
  };

  GenericTactileData(bool tactile_data_valid, int sample_frequency,
                     std::string manufacturer, std::string serial_number,
                     int software_version_current, int software_version_server,
                     bool software_version_modified, std::string pcb_version) :
    tactile_data_valid(tactile_data_valid),
    sample_frequency(sample_frequency),
    manufacturer(manufacturer),
    serial_number(serial_number),
    software_version_current(software_version_current),
    software_version_server(software_version_server),
    software_version_modified(software_version_modified),
    pcb_version(pcb_version)
  {
  };

  virtual ~GenericTactileData()
  {
  };

  bool tactile_data_valid;

  int which_sensor;
  int sample_frequency;
  std::string manufacturer;
  std::string serial_number;

  int software_version_current;
  int software_version_server;
  bool software_version_modified;

  std::string git_revision;

  /**
   * Parses the version string received
   *  from the tactiles and fill in the
   *  variables.
   *
   * @param raw_version The raw version string.
   */
  void set_software_version(char* raw_version)
  {
    // New Git format: \n\n 20 bytes Git revision + 1 byte for status check enable flag
    if (raw_version[0] == '\n' && raw_version[1] == '\n')
    {
      // Convert Git revision to hexadecimal long hash
      std::stringstream git_revision;
      for (int i = 2; i < 23; i++)
      {
        git_revision << std::setfill('0') << std::setw(2) << std::hex <<
          static_cast<int>(static_cast<uint8_t>(raw_version[i]));
      }
      this->git_revision = git_revision.str();
      return;
    }

    // Old Subversion format: number \n number \n Yes/No
    std::string version = raw_version;
    // split the string to fill the different versions
    std::vector<std::string> splitted_string;
    boost::split(splitted_string, version, boost::is_any_of("\n"));

    ROS_DEBUG("Tactile version: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
              static_cast<unsigned char>(version[0]), static_cast<unsigned char>(version[1]),
              static_cast<unsigned char>(version[2]), static_cast<unsigned char>(version[3]),
              static_cast<unsigned char>(version[4]), static_cast<unsigned char>(version[5]),
              static_cast<unsigned char>(version[6]), static_cast<unsigned char>(version[7]),
              static_cast<unsigned char>(version[8]), static_cast<unsigned char>(version[9]),
              static_cast<unsigned char>(version[10]), static_cast<unsigned char>(version[11]),
              static_cast<unsigned char>(version[12]), static_cast<unsigned char>(version[13]),
              static_cast<unsigned char>(version[14]), static_cast<unsigned char>(version[15]));
    if (splitted_string.size() >= 3)
    {
      software_version_current = convertToInt(splitted_string[0]);
      software_version_server = convertToInt(splitted_string[1]);

      if (splitted_string[2] == "No")
      {
        software_version_modified = false;
      }
      else
      {
        software_version_modified = true;
      }
    }
    else
    {
      ROS_ERROR("Incorrect tactile sensor version format");
      software_version_current = 0;
      software_version_server = 0;
      software_version_modified = false;
    }
  };

  /**
   * Formats the software version for the
   *  diagnostics.
   *
   * @return the formatted string
   */
  virtual std::string get_software_version()
  {
    // concatenate versions in a string.
    std::string full_version;

    std::stringstream ss;
    if (software_version_modified)
    {
      ss << "current: " << software_version_current << " / server: " << software_version_server << " / MODIFIED";
    }
    else
    {
      ss << "current: " << software_version_current << " / server: " << software_version_server << " / not modified";
    }

    full_version = ss.str();

    return full_version;
  };

  std::string pcb_version;

  inline double convertToInt(std::string const &s)
  {
    std::istringstream i(s);
    int x;
    if (!(i >> x))
    {
      x = -1;
    }
    return x;
  }
};

class PST3Data :
        public GenericTactileData
{
public:
  PST3Data() :
    GenericTactileData()
  {
  };

  PST3Data(const PST3Data &pst3) :
    GenericTactileData(pst3.tactile_data_valid, pst3.sample_frequency,
                       pst3.manufacturer, pst3.serial_number,
                       pst3.software_version_current,
                       pst3.software_version_server,
                       pst3.software_version_modified,
                       pst3.pcb_version),
    pressure(pst3.pressure),
    temperature(pst3.temperature),
    debug_1(pst3.debug_1),
    debug_2(pst3.debug_2),
    pressure_raw(pst3.pressure_raw),
    zero_tracking(pst3.zero_tracking),
    dac_value(pst3.dac_value)
  {
  };


  explicit PST3Data(const GenericTactileData &gtd) :
            GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                               gtd.manufacturer, gtd.serial_number,
                               gtd.software_version_current,
                               gtd.software_version_server,
                               gtd.software_version_modified,
                               gtd.pcb_version)
  {
  };

  ~PST3Data()
  {
  };
  int pressure;
  int temperature;

  int debug_1;
  int debug_2;

  int pressure_raw;
  int zero_tracking;

  int dac_value;

  /**
   * Formats the software version for the
   *  diagnostics.
   *
   * @return the formatted string
   */
  virtual std::string get_software_version()
  {
    // concatenate versions in a string.
    std::string full_version;

    std::stringstream ss;
    ss << "current: " << software_version_current;

    full_version = ss.str();

    return full_version;
  };
};

class BiotacData :
        public GenericTactileData
{
public:
  BiotacData() :
    GenericTactileData()
  {
    pac_buffer_ = boost::circular_buffer<int16_t>(pac_size_);
    pac_vector_.reserve(pac_size_);
  };

  BiotacData(const BiotacData &btac) :
    GenericTactileData(btac.tactile_data_valid, btac.sample_frequency,
                       btac.manufacturer, btac.serial_number,
                       btac.software_version_current,
                       btac.software_version_server,
                       btac.software_version_modified,
                       btac.pcb_version),
    pac0(btac.pac0),
    pac1(btac.pac1),
    pdc(btac.pdc),
    tac(btac.tac),
    tdc(btac.tdc)
  {
    electrodes = std::vector<int16_t>(btac.electrodes);
    pac_vector_ = std::vector<int16_t>(btac.pac_vector_);
    pac_buffer_ = boost::circular_buffer<int16_t>(btac.pac_buffer_);
  };

  explicit BiotacData(const GenericTactileData &gtd) :
    GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                       gtd.manufacturer, gtd.serial_number,
                       gtd.software_version_current,
                       gtd.software_version_server,
                       gtd.software_version_modified,
                       gtd.pcb_version)
  {
    pac_buffer_ = boost::circular_buffer<int16_t>(pac_size_);
    pac_vector_.reserve(pac_size_);
  };

  ~BiotacData()
  {
  };

  std::vector<int16_t> get_pac(bool consume = false)
  {
    pac_vector_.clear();
    pac_vector_.insert(pac_vector_.begin(), pac_buffer_.begin(), pac_buffer_.end());
    if (consume)
    {
      pac_buffer_.clear();
    }
    return pac_vector_;
  }

  std::vector<int16_t> consume_pac()
  {
    return get_pac(true);
  }

  int pac0;  // always there, in word[0] and 1; int16u (2kHz)
  int pac1;  // int16u

  int pdc;  // int16u in word[2]

  int tac;  // int16u in word[2]
  int tdc;  // int16u in word[2]
  std::vector<int16_t> electrodes;  // int16u in word[2]
  boost::circular_buffer<int16_t> pac_buffer_;  // 2kHz history of int16u/word[2] values. Capacity of 270 samples is
                                                // 135ms history
  static const size_t pac_size_ = 270;

private:
  std::vector<int16_t> pac_vector_;
};

class UBI0Data :
        public GenericTactileData
{
public:
  UBI0Data() :
    GenericTactileData()
  {
  };

  UBI0Data(const UBI0Data &ubi0) :
    GenericTactileData(ubi0.tactile_data_valid, ubi0.sample_frequency,
                       ubi0.manufacturer, ubi0.serial_number,
                       ubi0.software_version_current,
                       ubi0.software_version_server,
                       ubi0.software_version_modified,
                       ubi0.pcb_version)
  {
    for (unsigned int i = 0; i < ubi0.distal.size(); i++)
    {
      distal[i] = ubi0.distal[i];
    }
    for (unsigned int i = 0; i < ubi0.middle.size(); i++)
    {
      middle[i] = ubi0.middle[i];
    }
    for (unsigned int i = 0; i < ubi0.proximal.size(); i++)
    {
      proximal[i] = ubi0.proximal[i];
    }
  };

  explicit UBI0Data(const GenericTactileData &gtd) :
            GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                               gtd.manufacturer, gtd.serial_number,
                               gtd.software_version_current,
                               gtd.software_version_server,
                               gtd.software_version_modified,
                               gtd.pcb_version)
  {
  };

  ~UBI0Data()
  {
  };


  boost::array<uint16_t, 12ul> distal;
  boost::array<uint16_t, 4ul> middle;
  boost::array<uint16_t, 4ul> proximal;
};

class UBI0PalmData
{
public:
  UBI0PalmData()
  {
  };

  UBI0PalmData(const UBI0PalmData &ubi0)
  {
    for (unsigned int i = 0; i < ubi0.palm.size(); i++)
    {
      palm[i] = ubi0.palm[i];
    }
  };

  ~UBI0PalmData()
  {
  };

  boost::array<uint16_t, 16ul> palm;
};

class MSTData :
        public GenericTactileData
{
public:
  MSTData() :
    GenericTactileData()
  {
  };

  explicit MSTData(const GenericTactileData &gtd) :
    GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                       gtd.manufacturer, gtd.serial_number,
                       gtd.software_version_current,
                       gtd.software_version_server,
                       gtd.software_version_modified,
                       gtd.pcb_version)
  {
  };

  ~MSTData()
  {
  };

  std::vector<geometry_msgs::Point> magnetic_data;
  std::vector<float> temperature_data;
  int8_t status_check;
};

struct AllTactileData
{
  std::string type;
  BiotacData biotac;
  PST3Data pst;
  MSTData mst;
  UBI0Data ubi0;
};
}  // namespace tactiles

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
