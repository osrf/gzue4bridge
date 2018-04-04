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


#include "GzNode.h"
#include "GzWSClient.h"

#include "Json.h"
#include "JsonUtilities.h"

#include <chrono>

#include <functional>
#include <mutex>
#include <thread>

/// \brief Private data for the GzNode class
class gazebo::ue4::GzNodePrivate
{
  /// \brief Subscription handler. Store map of topic to subscriber callback
  public: std::map<std::string, std::vector<
      std::function<void (TSharedPtr<FJsonObject>)>>> subHandler;

  /// \brief Websocket client for connecting to gzbridge
  public: GzWSClient gzWSClient;

  /// \brief Incoming messages from gzbridge
  public: std::vector<std::string> messages;

  /// \brief Outgoing messages to be sent to gzbridge
  public: std::vector<std::string> outgoingMessages;

  /// \brief Mutex to protect incoming messages
  public: std::mutex msgMutex;

  /// \brief Mutex to protect outgoing messages
  public: std::mutex outMutex;

  /// \brief Mutex to protect subscription handlers
  public: std::mutex handlerMutex;

  /// \brief Thread for processing incoming and outgoing messages
  public: std::thread *runThread = nullptr;

  /// \brief True to stop the node
  public: bool stop = false;
};

using namespace gazebo;
using namespace ue4;

//////////////////////////////////////////////////
GzNode::GzNode()
  : dataPtr(new GzNodePrivate)
{
}

//////////////////////////////////////////////////
GzNode::~GzNode()
{
  this->dataPtr->stop = true;

  if (this->dataPtr->runThread)
  {
    this->dataPtr->runThread->join();
    delete this->dataPtr->runThread;
  }
}

//////////////////////////////////////////////////
void GzNode::Init()
{
  this->dataPtr->runThread = new std::thread(
      std::bind(&GzNode::RunLoop, this));
}

//////////////////////////////////////////////////
void GzNode::MessageCallback(const std::string &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
  this->dataPtr->messages.push_back(_msg);
}

//////////////////////////////////////////////////
void GzNode::RunLoop()
{
  this->dataPtr->gzWSClient.SetMessageCallback(
      std::bind(&GzNode::MessageCallback, this, std::placeholders::_1));

  this->dataPtr->gzWSClient.Connect("ws://localhost:8080");

  while (!this->dataPtr->stop)
  {
    if (!this->dataPtr->gzWSClient.IsOpen())
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(this->dataPtr->handlerMutex);
      if (this->dataPtr->subHandler.empty())
        continue;
    }

    // process incoming messages from gzbridge
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
      for (auto &m : this->dataPtr->messages)
      {
        TSharedPtr<FJsonObject> jsonParsed;
        TSharedRef<TJsonReader<TCHAR>> jsonReader =
            TJsonReaderFactory<TCHAR>::Create(m.c_str());
        if (FJsonSerializer::Deserialize(jsonReader, jsonParsed))
        {
          FString opValue = jsonParsed->GetStringField("op");
          if (opValue == "publish")
          {
            FString topicValue = jsonParsed->GetStringField("topic");
            TSharedPtr<FJsonObject> msgObj = jsonParsed->GetObjectField("msg");
            // UE_LOG(LogTemp, Warning, TEXT("op: %s, topic: %s"), *opValue, *topicValue);
            std::lock_guard<std::mutex> lock2(this->dataPtr->handlerMutex);
            std::string str = TCHAR_TO_UTF8(*topicValue);
            auto it = this->dataPtr->subHandler.find(str);
            if (it != this->dataPtr->subHandler.end())
            {
              for (auto &cb : it->second)
              {
                cb(msgObj);
              }
            }
          }
        }
      }
      this->dataPtr->messages.clear();
    }

    // process outgoing messages to gzbridge
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->outMutex);
      for (auto &m : this->dataPtr->outgoingMessages)
      {
        this->dataPtr->gzWSClient.Send(m);
      }
      this->dataPtr->outgoingMessages.clear();
    }
  }
}

//////////////////////////////////////////////////
void GzNode::Subscribe(const std::string &_topic,
   std::function<void (TSharedPtr<FJsonObject>)> _cb)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->handlerMutex);
  this->dataPtr->subHandler[_topic].push_back(_cb);

  static int subId = 0;
  std::string id = "subscribe:" + _topic + ":" + std::to_string(subId++);
  std::stringstream subReqStr;
  subReqStr << "{"
            << "\"op\":\"subscribe\", "
            << "\"id\":" << "\"" << id << "\", "
            << "\"topic\":" << "\"" << _topic << "\""
            << "}";

  this->Send(subReqStr.str());
}

//////////////////////////////////////////////////
void GzNode::Publish(const std::string &_topic, TSharedPtr<FJsonObject> _msg)
{
  TSharedPtr<FJsonObject> json = MakeShareable(new FJsonObject);
  json->SetStringField("op", "publish");
  json->SetStringField("topic", _topic.c_str());
  json->SetObjectField("msg", _msg);

  FString outputStr;
  TSharedRef<TJsonWriter<>> writer = TJsonWriterFactory<>::Create(&outputStr);
  FJsonSerializer::Serialize(json.ToSharedRef(), writer);
  std::string msg = TCHAR_TO_UTF8(*outputStr);
//  if (_topic != "~/model/modify")
  this->Send(msg);
}

//////////////////////////////////////////////////
void GzNode::Send(const std::string &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->outMutex);
  this->dataPtr->outgoingMessages.push_back(_msg);
}
