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
    class FGzIfacePrivate;

    /// \brief An interface to Gazebo simulation server
    class FGzIface
    {
      /// \brief Constructor
      private: FGzIface();

      /// \brief Destructor
    	public: ~FGzIface();

      /// \brief Get an instance of this singleton object
      /// \return GZIface instance
    	public: static FGzIface &Instance();

      /// \brief Initial Gazebo interface by subscribing to Gazebo topics
    	public: void Init();

      /// \brief Shutdown Gazebo interface.
    	public: void Shutdown();

      /// \brief Get a pointer to the game world
      /// \return Pointer to game world. NULL if a world is not available, e.g.
      /// when game is not runnning.
    	public: UWorld *GameWorld();

      /// \brief Overrides FTickableGameObject::Tick
      /// Performs synchronization of Unreal and Gazebo
      public: void Tick(float _delta);

      /// \brief Scene message callback
      /// \param[in] _json JSON message
      private: void OnSceneMsg(TSharedPtr<FJsonObject> _json);

      /// \brief Model message callback
      /// \param[in] _json JSON message
      private: void OnModelMsg(TSharedPtr<FJsonObject> _json);

      /// \brief Pose message callback
      /// \param[in] _json JSON message
      private: void OnPoseMsg(TSharedPtr<FJsonObject> _json);

      /// \brief Model message callback
      /// \param[in] _json JSON message
      /// \return True of model is created
      private: bool CreateModelFromMsg(TSharedPtr<FJsonObject> _json);

      /// \brief Pose message callback
      /// \param[in] _json JSON message
      /// \return True pose update is successful
      private: bool UpdatePoseFromMsg(TSharedPtr<FJsonObject> _json);

      /// \brief Advertise to Gazebo that a new static mesh actor is available
      /// \param[in] _actor New static mesh actor
      /// \return True if the actor advertise operation is successful
    	private: bool AdvertiseStaticMeshActor(AActor *_actor);

      /// \brief Advertise to Gazebo that a new skeletal mesh actor is available
      /// \param[in] _actor New skeletal actor
      /// \return True if the advertise operation is successful
    	private: bool AdvertiseSkeletalMeshActor(AActor *_actor);

      /// \brief Publish to Gazebo the pose of unreal's skeletal mesh actors
      /// \return True if the pose are published
    	private: bool PublishSkeletalMeshActor(AActor *_actor);

      /// \brief Request Gazebo to step
      /// \param[in] _steps Number of steps to take
      /// \return True if the request is sent successfully
    	private: bool StepGz(const int _steps);

      /// \brief Set whether to pause unreal actors. This is done by setting
      /// the custom time dilation.
      /// \param[in] _paused True to pause, false to unpause
    	private: void SetActorsPaused(const bool _paused);

      /// \brief Reconstruct Gazebo scene in Unreal
      /// \return True if the scene is created.
    	private: bool GzToUE4Scene();

      /// \brief Publish messages to create Unreal models in Gazebo
      /// \return True if the messages are published
    	private: bool UE4ToGzModelSync();

      /// \brief Create Gazebo models in Unreal
      /// \return True if the models are created.
    	private: bool GzToUE4ModelSync();

      /// \brief Publish messages to update Unreal model pose in Gazebo
    	private: void UE4ToGzPoseSync();

      /// \brief Update Unreal models' pose based on Gazebo pose messages.
    	private: void GzToUE4PoseSync();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<FGzIfacePrivate> dataPtr;
    };
  }
}
