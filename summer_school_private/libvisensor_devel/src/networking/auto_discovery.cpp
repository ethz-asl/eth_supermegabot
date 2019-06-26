/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ifaddrs.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/timer.hpp>

#include "networking/auto_discovery.hpp"

#define SENSOR_SEARCHES 2

using boost::asio::deadline_timer;
using boost::asio::ip::udp;

namespace visensor
{

AutoDiscovery::AutoDiscovery(unsigned short int port)
: socket_(io_service_),
  port_(port){
  data_ = new char[10];
}

AutoDiscovery::~AutoDiscovery() {
  socket_.close();
  delete data_;
}

class client
{
 public:
  client(const udp::endpoint& listen_endpoint)
 : socket_(io_service_, listen_endpoint),
   deadline_(io_service_)
 {
    // No deadline is required until the first socket operation is started. We
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);

    // Start the persistent actor that checks for deadline expiry.
    check_deadline();
 }

  ~client()
  {
    socket_.close();
  }

  std::size_t receive_from(const boost::asio::mutable_buffer& buffer, boost::asio::ip::udp::endpoint& sender,
                           boost::posix_time::time_duration timeout, boost::system::error_code& ec)
  {
    // Set a deadline for the asynchronous operation.
    deadline_.expires_from_now(timeout);

    // Set up the variables that receive the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    ec = boost::asio::error::would_block;
    std::size_t length = 0;

    // Start the asynchronous operation itself. The handle_receive function
    // used as a callback will update the ec and length variables.
    socket_.async_receive_from(boost::asio::buffer(buffer), sender,
                               boost::bind(&client::handle_receive, _1, _2, &ec, &length));

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    return length;
  }

 private:
  void check_deadline()
  {
    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (deadline_.expires_at() <= deadline_timer::traits_type::now())
    {
      // The deadline has passed. The outstanding asynchronous operation needs
      // to be cancelled so that the blocked receive() function will return.
      //
      // Please note that cancel() has portability issues on some versions of
      // Microsoft Windows, and it may be necessary to use close() instead.
      // Consult the documentation for cancel() for further information.
      socket_.cancel();

      // There is no longer an active deadline. The expiry is set to positive
      // infinity so that the actor takes no action until a new deadline is set.
      deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Put the actor back to sleep.
    deadline_.async_wait(boost::bind(&client::check_deadline, this));
  }

  static void handle_receive(
      const boost::system::error_code& ec, std::size_t length,
      boost::system::error_code* out_ec, std::size_t* out_length)
  {
    *out_ec = ec;
    *out_length = length;
  }

 private:
  boost::asio::io_service io_service_;
  udp::socket socket_;
  deadline_timer deadline_;
};

std::vector<AutoDiscovery::IpSetting> AutoDiscovery::getIpList() {

  // retrieve the current interfaces
  ifaddrs *interfaces = NULL;
  int rc = getifaddrs(&interfaces);
  VISENSOR_ASSERT_COND(rc==-1, "Could not obtain list of network interfaces.");

  //get ip address for all active IPv4 interfaces
  std::vector<IpSetting> ip_addresses;

  for (ifaddrs *ifa = interfaces; ifa != NULL; ifa = ifa->ifa_next)
  {
    if (ifa->ifa_addr == NULL)
      continue;

    if (ifa->ifa_addr->sa_family == AF_INET)
    {
      //get interfaces ip address
      char host[NI_MAXHOST];
      int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      VISENSOR_ASSERT_COND(s!=0, "Could not obtain ipaddr for network interface.");
      char mask[NI_MAXHOST];
      s = getnameinfo(ifa->ifa_netmask, sizeof(struct sockaddr_in), mask, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      VISENSOR_ASSERT_COND(s!=0, "Could not obtain netmask for network interface.");

      // VISENSOR_DEBUG("\tAutodiscovery found interface: <%s; %s>\n", host, mask);
      IpSetting ip;
      ip.ip = boost::asio::ip::address_v4::from_string(host);
      ip.mask = boost::asio::ip::address_v4::from_string(mask);
      ip_addresses.push_back( ip );
    }
  }

  return ip_addresses;
}

ViDeviceList AutoDiscovery::findSensor()
{
  SensorSet sensor_set;

  try
  {
    boost::system::error_code error;
    socket_.open(boost::asio::ip::udp::v4(), error);

    // get list of interfaces
    std::vector<AutoDiscovery::IpSetting> ip_addresses = getIpList();

    VISENSOR_DEBUG("Autodiscovery is searching for sensor on interfaces:\n");

    for(uint j = 0; j<ip_addresses.size(); j++)
      VISENSOR_DEBUG("\t %d: IP: %s NETMASK: %s\n", j, ip_addresses[j].ip.to_string().c_str(), ip_addresses[j].mask.to_string().c_str());

    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.set_option(boost::asio::socket_base::broadcast(true));

    //      boost::asio::ip::udp::endpoint senderEndpoint(boost::asio::ip::address_v4::broadcast(), port_);

    std::string message ="Hello sensor";

    // wait for answer
    boost::asio::ip::address ipAddr = boost::asio::ip::address_v4::any();
    boost::asio::ip::udp::endpoint listen_endpoint(ipAddr, 13775);
    client c(listen_endpoint);

    //send requests
    for (int i = 0; i<SENSOR_SEARCHES; i++)
    {
      for(uint j = 0; j<ip_addresses.size(); j++)
      {
        //VISENSOR_DEBUG("send to: %s with netmask: %s\n", ip_addresses[j].to_string().c_str(), boost::asio::ip::address_v4::netmask(ip_addresses[j]).to_string().c_str());
        boost::asio::ip::udp::endpoint senderEndpoint(boost::asio::ip::address_v4::broadcast(ip_addresses[j].ip, ip_addresses[j].mask), port_);
        try {
          socket_.send_to(boost::asio::buffer(message), senderEndpoint);
        } catch(std::exception const &ex)
        {
          //ignore and try next interface... (eg. vpn interfaces will throw...)
        }
      }
    }

    //receive requests
    boost::system::error_code ec;
    char data[12];

    //listen to all sensor responses
    while(ec != boost::asio::error::operation_aborted)
    {
      c.receive_from(boost::asio::buffer(data), sensor_endpoint_, boost::posix_time::milliseconds(15), ec);
      sensor_set.insert( sensor_endpoint_.address().to_string() );
    }
  }
  catch (std::exception& e)
  {
    VISENSOR_DEBUG("Autodiscovery exception: %s", e.what());
  }

  //convert sensor set to vector
  ViDeviceList sensor_list(sensor_set.size());
  std::copy(sensor_set.begin(), sensor_set.end(), sensor_list.begin());

  //delete null-connections
  for (unsigned int i=0; i<sensor_list.size(); i++)
  {
    if(sensor_list[i] == "0.0.0.0")
      sensor_list.erase(sensor_list.begin()+i);
  }

  //debug output
  VISENSOR_DEBUG("Autodiscovery found the following sensors:\n");
  if( !sensor_list.empty() )
  {
    for (unsigned int i=0; i<sensor_list.size(); i++)
      VISENSOR_DEBUG("\t %u: %s\n", i, sensor_list[i].c_str());

  } else {
    VISENSOR_DEBUG("\t no sensors found!\n");
  }

  return sensor_list;
}

} //namespace visensor
