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

#include "Engine/GameInstance.h"

#include "GzIfaceGameInstance.generated.h"

// forward declaration
class UGzIfaceGameInstance;

/// \brief Tick function for our custom game instance.
/// A game instance object does not tick so we are need to hook it up with this
/// custom tick tick function.
class FGzTickFunction : public FTickFunction
{
  /// \brief Execute tick function
  public: virtual void ExecuteTick(
     float DeltaTime,
     ELevelTick TickType,
     ENamedThreads::Type CurrentThread,
     const FGraphEventRef& MyCompletionGraphEvent) override;

  /// \brief Pointer to our custom game instance
  public: UGzIfaceGameInstance *target = nullptr;
};


UCLASS()
class UGzIfaceGameInstance : public UGameInstance
{
	GENERATED_BODY()

  /// \brief Constructor
  public: UGzIfaceGameInstance(const FObjectInitializer& ObjectInitializer);

  /// \brief Initialize game instance.
  /// This creates connection to gazebo.
  public: void Init() override;

  /// \brief Shutdown game instance
  /// This removes connection to gazebo.
  public: void Shutdown() override;

  /// \brief Global tick function. This is called in the PrePhysics tick group
  /// before the actors are updated.
  /// \param[in] _delta Game time delta.
  public: void CustomTick(float _delta);

  /// \brief Custom tick function
  FGzTickFunction preTickFunction;
};
