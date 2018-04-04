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

#include "GzIfaceGameInstance.h"

//////////////////////////////////////////////////
void FGzTickFunction::ExecuteTick(
     float DeltaTime,
     ELevelTick TickType,
     ENamedThreads::Type CurrentThread,
     const FGraphEventRef& MyCompletionGraphEvent)
{
  if (this->target)
  {
    // FScopeCycleCounterUObject ActorScope(Target);
    this->target->CustomTick(DeltaTime);
  }
}

//////////////////////////////////////////////////
UGzIfaceGameInstance::UGzIfaceGameInstance(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
}

//////////////////////////////////////////////////
void UGzIfaceGameInstance::Init()
{
  UWorld *world = this->GetWorld();
  if (world == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Error getting world"));
    return;
  }

  this->preTickFunction.TickGroup = TG_PrePhysics;
  this->preTickFunction.bCanEverTick = true;
  this->preTickFunction.target = this;
  this->preTickFunction.RegisterTickFunction(world->PersistentLevel);

  FGzIface::Instance().Init();
}

//////////////////////////////////////////////////
void UGzIfaceGameInstance::Shutdown()
{
  FGzIface::Instance().Shutdown();
}

//////////////////////////////////////////////////
void UGzIfaceGameInstance::CustomTick(float _delta)
{
  FGzIface::Instance().Tick(_delta);
}
