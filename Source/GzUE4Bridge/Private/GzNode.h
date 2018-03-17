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

#include "CoreMinimal.h"

namespace gazebo
{
  namespace ue4
  {
    // forward declaration
    class GzNodePrivate;

    /// \brief A class to handle topic subscription and publication
    class GzNode
    {
      /// \brief Constructor
      /// \param[in] _client Websocket client
      public: GzNode();

      /// \brief Destructor
    	public: ~GzNode();

      /// \brief Initialize the node.
    	public: void Init();

      /// \brief Subscribe to a topic
      /// \param[in] _topic Topic to subscribe to
      /// \param[in] _cb Callback to execute when a message is received on
      /// the topic
      public: void Subscribe(const std::string &_topic,
            std::function<void (TSharedPtr<FJsonObject>)> _cb);

      /// \brief Public to a topic
      /// \param[in] _topic Topic to publish to
      /// \param[in] _msg Msg to send
      public: void Publish(const std::string &_topic, TSharedPtr<FJsonObject> _msg);

      /// \brief Callback when a message is received on websocket client
      public: void MessageCallback(const std::string &_msg);

      /// \brief Send a json message over websocket
      /// \param[in] _msg Json message in string format
      private: void Send(const std::string &_msg);

      /// \brief Loop for message processing
      private: void RunLoop();

      /// \internal
      /// \brief Pointer to private data;
      private: std::unique_ptr<GzNodePrivate> dataPtr;

    };
  }
}
