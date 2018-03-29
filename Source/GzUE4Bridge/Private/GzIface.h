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
#include "Tickable.h"

namespace gazebo
{
  namespace ue4
  {
    // forward declaration
    class FGzIfacePrivate;

    /// \brief A interface to gazebo simulation server
    class FGzIface : public FTickableGameObject
    {
      /// \brief Constructor
      private: FGzIface();

      /// \brief Destructor
    	public: ~FGzIface();

    	public: static FGzIface &Instance();

    	public: void Init();
    	public: void Sync();
    	public: void ShutDown();

    	public: UWorld *GameWorld();

      protected: void Tick(float DeltaTime) override;
      protected: bool IsTickable() const override;
      protected: bool IsTickableInEditor() const override;
      protected: bool IsTickableWhenPaused() const override;
      protected: TStatId GetStatId() const override;



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

    	public: bool SyncStaticMeshActor(AActor *_actor);
    	public: bool SyncSkeletalMeshActor(AActor *_actor);
    	public: bool UpdateSkeletalMeshActor(AActor *_actor);
    	public: bool StepGz();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<FGzIfacePrivate> dataPtr;
    };
  }
}
