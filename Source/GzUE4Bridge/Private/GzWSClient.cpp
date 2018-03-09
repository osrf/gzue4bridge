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

#include "GzWSClient.h"

#define _WEBSOCKETPP_CPP11_STL_
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <functional>
#include <mutex>
#include <thread>


using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

/// \brief Private data for the GzWSClient class
class gazebo::ue4::GzWSClientPrivate
{
  /// \brief Websocket client.
  public: wsclient client;

  /// \brief Websocket client connection.
  public: wsclient::connection_ptr connection;

  /// \brief Websocket server url
  public: std::string url;

  /// \brief Callback function when a message is received
  public: std::function<void (const std::string &)> msgCallback;

  /// \brief Websocket client thread
  public: std::thread *wsThread = nullptr;

  /// \brief Flag to indicate if connection is open
  public: bool open = false;

  /// \brief Mutex to protect connection open flag
  public: std::mutex conMutex;
};

using namespace gazebo;
using namespace ue4;

//////////////////////////////////////////////////
GzWSClient::GzWSClient()
  : dataPtr(new GzWSClientPrivate)
{
}

//////////////////////////////////////////////////
GzWSClient::~GzWSClient()
{

  if (this->dataPtr->wsThread)
  {
    this->dataPtr->client.stop_perpetual();
    websocketpp::lib::error_code ec;
    this->dataPtr->client.close(this->dataPtr->connection->get_handle(),
        websocketpp::close::status::going_away, "", ec);
    if (ec)
    {
      UE_LOG(LogTemp, Warning, TEXT("Error closing connection: %s"),
          ec.message().c_str());
    }

    this->dataPtr->wsThread->join();
    delete this->dataPtr->wsThread;
    this->dataPtr->wsThread = nullptr;
  }
}

//////////////////////////////////////////////////
void GzWSClient::OnMessage(websocketpp::connection_hdl _hdl,
    wsmessage_ptr _msg)
{
  if (!this->dataPtr->msgCallback)
    return;

  std::cout << "on_message called with hdl: " << _hdl.lock().get()
            << " and message: " << _msg->get_payload()
            << std::endl;
  this->dataPtr->msgCallback(_msg->get_payload());
}

//////////////////////////////////////////////////
void GzWSClient::OnOpen(websocketpp::connection_hdl _hdl)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->conMutex);
  this->dataPtr->open = true;
}

//////////////////////////////////////////////////
bool GzWSClient::IsOpen() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->conMutex);
  return this->dataPtr->open;
}

//////////////////////////////////////////////////
void GzWSClient::Send(const std::string &_msg)
{
  if (!this->dataPtr->connection)
    return;

  websocketpp::lib::error_code ec;
  this->dataPtr->client.send(
      this->dataPtr->connection->get_handle(),
      _msg, websocketpp::frame::opcode::text, ec);

  if (ec)
  {
    UE_LOG(LogTemp, Warning, TEXT("Error sending message: %s"),
        ec.message().c_str());
    return;
  }
}

//////////////////////////////////////////////////
void GzWSClient::SetMessageCallback(
    std::function<void (const std::string &)> _cb)
{
  this->dataPtr->msgCallback = _cb;
}

//////////////////////////////////////////////////
void GzWSClient::Connect(const std::string &_url)
{
  if (_url.empty())
  {
    UE_LOG(LogTemp, Warning, TEXT("Empty websocket URL. Not Connecting"));
    return;
  }

  if (this->dataPtr->wsThread)
  {
    UE_LOG(LogTemp, Warning, TEXT("Websocket client already started"));
    return;
  }

  this->dataPtr->url = _url;

  this->dataPtr->wsThread = new std::thread(
      std::bind(&GzWSClient::Run, this));
}

//////////////////////////////////////////////////
void GzWSClient::Run()
{
  try
  {
    // Set logging to be pretty verbose (everything except message payloads)
    this->dataPtr->client.set_access_channels(websocketpp::log::alevel::all);
    this->dataPtr->client.clear_access_channels(
        websocketpp::log::alevel::frame_payload);

    // Initialize ASIO
    this->dataPtr->client.init_asio();
    this->dataPtr->client.start_perpetual();

    // Register our message handler
    this->dataPtr->client.set_message_handler(bind(&GzWSClient::OnMessage,
        this, ::_1, ::_2));

    this->dataPtr->client.set_open_handler(bind(&GzWSClient::OnOpen,
        this, ::_1));


    websocketpp::lib::error_code ec;
    this->dataPtr->connection =
        this->dataPtr->client.get_connection(this->dataPtr->url, ec);

    if (ec)
    {
      UE_LOG(LogTemp, Warning,
          TEXT("Websocket client could not create connection because: %s"),
          ec.message().c_str());
      return;
    }

    // Note that connect here only requests a connection. No network messages are
    // exchanged until the event loop starts running in the next line.
    this->dataPtr->client.connect(this->dataPtr->connection);

    // Start the ASIO io_service run loop
    // this will cause a single connection to be made to the server. c.run()
    // will exit when this connection is closed.
    this->dataPtr->client.run();
  }
  catch (websocketpp::exception const & e)
  {
    UE_LOG(LogTemp, Warning, TEXT("Websocketpp exception %s"), e.what());
  }
}
