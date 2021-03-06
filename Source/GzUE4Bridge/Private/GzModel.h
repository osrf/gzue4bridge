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

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GzModel.generated.h"

UCLASS()
class AGzModel : public AActor
{
	GENERATED_BODY()

	/// \brief Sets default values for this actor's properties
  public: AGzModel(const FObjectInitializer& ObjectInitializer);

	/// \brief Called every frame
  public: virtual void Tick(float DeltaTime) override;

  /// \brief Load the model from a json object message
  public: void Load(TSharedPtr<FJsonObject> _json);

	/// \brief Called when the game starts or when spawned
  protected: virtual void BeginPlay() override;

	/// \brief Load link from json message
  /// \param[in] _json json message
  /// \param[in] _parent Parent of link
  private: void LoadLink(TSharedPtr<FJsonObject> _json,
        USceneComponent *_parent);

	/// \brief Load visual from json message
  /// \param[in] _json json message
  /// \param[in] _parent Parent of visual
  private: void LoadVisual(TSharedPtr<FJsonObject> _json,
        USceneComponent *_parent);

	/// \brief Load geometry from json message
  /// \param[in] _json json message
  /// \param[in] _parent Parent of geometry
  private: void LoadGeometry(TSharedPtr<FJsonObject> _json,
        USceneComponent *_parent);

  UPROPERTY(EditAnywhere)
  float RunningTime;
};
