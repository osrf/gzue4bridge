/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#pragma once

#include <memory>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include "CoreMinimal.h"


typedef websocketpp::client<websocketpp::config::asio_client> wsclient;
// pull out the type of messages sent by our config
typedef websocketpp::config::asio_client::message_type::ptr wsmessage_ptr;

namespace gazebo
{
  namespace ue4
  {
    // forward declaration
    class GzWSClientPrivate;

    class GzWSClient
    {
      /// \brief Constructor
      public: GzWSClient();

      /// \brief Destructor
    	public: ~GzWSClient();

      /// \brief Connect to the specified url
      /// \param[in] _url Websocket url address
      public: void Connect(const std::string &_url);

      /// \brief Send a json message
      /// \param[in] _msg Json message in string format
      public: void Send(const std::string &_msg);

      /// \brief Callback when a message is received
      public: void SetMessageCallback(
            std::function<void (const std::string &)> _cb);

      /// \brief Get whether the connection is open.
      /// \return True if connection is open
      public: bool IsOpen() const;

      /// \brief Callback when a message is received
      /// \param[in] _hdl Connection handle
      private: void OnMessage(websocketpp::connection_hdl _hdl,
            wsmessage_ptr _msg);

      /// \brief Callback when a connection is open
      /// \param[in] _hdl Connection handle
      private: void OnOpen(websocketpp::connection_hdl _hdl);

      /// \brief Thread for running the websocket client
      private: void Run();

      /// \internal
      /// \brief Pointer to private data;
      private: std::unique_ptr<GzWSClientPrivate> dataPtr;
    };
  }
}
