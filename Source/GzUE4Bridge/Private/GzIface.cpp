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

#include "GzIface.h"

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "Runtime/Engine/Classes/Engine/GameEngine.h"
#include "Editor/UnrealEd/Classes/Editor/EditorEngine.h"
#include "EngineUtils.h"
#include "Json.h"
#include "JsonUtilities.h"

#include "GzNode.h"
#include "GzModel.h"
#include "GzUtil.h"

/// \brief Private data for the FGzIface class
class gazebo::ue4::FGzIfacePrivate
{
  /// \brief Node for communicating with gazebo bridge
  public: GzNode *node = nullptr;

  /// \brief True if gazebo interface has been initialized
  public: bool initialized = false;

  /// \brief Mutex to protect connection open flag
  public: std::mutex msgMutex;

  /// \brief A list actors in the world.
  public: std::map<FString, AActor *> actors;

  /// \brief A list of scene msgs.
  public: std::vector<TSharedPtr<FJsonObject>> sceneMsgs;

  /// \brief A list of model msgs.
  public: std::vector<TSharedPtr<FJsonObject>> modelMsgs;

  /// \brief A list of pose msgs.
  public: std::vector<TSharedPtr<FJsonObject>> poseMsgs;

};

using namespace gazebo;
using namespace ue4;

//////////////////////////////////////////////////
FGzIface &FGzIface::Instance()
{
  static FGzIface iface;
  return iface;
}

//////////////////////////////////////////////////
FGzIface::FGzIface()
  : dataPtr(new FGzIfacePrivate)
{
}

//////////////////////////////////////////////////
FGzIface::~FGzIface()
{
  this->ShutDown();
}

//////////////////////////////////////////////////
void FGzIface::Init()
{
  if (this->dataPtr->initialized)
    return;

  this->dataPtr->node = new GzNode();
  this->dataPtr->node->Init();
  this->dataPtr->node->Subscribe("~/scene", std::bind(&FGzIface::OnSceneMsg,
      this, std::placeholders::_1));
  this->dataPtr->node->Subscribe("~/model/info", std::bind(&FGzIface::OnModelMsg,
      this, std::placeholders::_1));
  this->dataPtr->node->Subscribe("~/pose/info", std::bind(&FGzIface::OnPoseMsg,
      this, std::placeholders::_1));

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void FGzIface::Sync()
{
  FString fname;
  for (TActorIterator<AActor> it(this->GameWorld()); it; ++it)
  {
    // store all existing actors
    // TODO filter out some unwanted ones?
    this->dataPtr->actors[it->GetName()] = *it;
  }
}

//////////////////////////////////////////////////
void FGzIface::ShutDown()
{
  delete this->dataPtr->node;
  this->dataPtr->node = nullptr;
  this->dataPtr->initialized = false;

  this->dataPtr->actors.clear();
}

//////////////////////////////////////////////////
void FGzIface::OnSceneMsg(TSharedPtr<FJsonObject> _json)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
  this->dataPtr->sceneMsgs.push_back(_json);
}

//////////////////////////////////////////////////
void FGzIface::OnModelMsg(TSharedPtr<FJsonObject> _json)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
  this->dataPtr->modelMsgs.push_back(_json);
}

//////////////////////////////////////////////////
void FGzIface::OnPoseMsg(TSharedPtr<FJsonObject> _json)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
  this->dataPtr->poseMsgs.push_back(_json);
}

//////////////////////////////////////////////////
bool FGzIface::CreateModelFromMsg(TSharedPtr<FJsonObject> _json)
{
  FString modelName = _json->GetStringField("name");
  if (this->dataPtr->actors.find(modelName) != this->dataPtr->actors.end())
    return false;

  FVector pos(0, 0, 0);
  FRotator rot(0, 0, 0);
  FActorSpawnParameters spawnParams;
  spawnParams.Name = FName(*modelName);
  AGzModel* model =
      this->GameWorld()->SpawnActor<AGzModel>(pos, rot, spawnParams);
  model->Load(_json);
  this->dataPtr->actors[model->GetName()] = model;
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::UpdatePoseFromMsg(TSharedPtr<FJsonObject> _json)
{
  FString name = _json->GetStringField("name");
  // identify which type of entity (model, link, or visual) this pose is
  // associated to. Do this by looking at the name.
  // gazebo naming convention
  //   model_name::link_name::visual::name

  // get model name
  FString modelName = name;
  std::string nameStr(TCHAR_TO_UTF8(*name));
  size_t idx = nameStr.find("::");
  bool isModel = true;
  if (idx != std::string::npos)
  {
    std::string n = nameStr.substr(0, idx);
    modelName = FString(n.c_str());
    isModel = false;
  }

  // find model
  auto it = this->dataPtr->actors.find(modelName);
  if (it == this->dataPtr->actors.end())
    return false;

  FVector pos;
  FRotator rot;
  GzUtil::ParsePose(_json, pos, rot);

  if (isModel)
  {
    it->second->SetActorLocation(GzUtil::CoordTransform(pos));
    it->second->SetActorRotation(GzUtil::CoordTransform(rot));
    return true;
  }

  // find link / visual and set its pose
  TArray<USceneComponent *> sceneComps;
  it->second->GetComponents(sceneComps);
  if (sceneComps.Num() <= 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("model does not contain components"));
    return false;
  }

  for (auto c : sceneComps)
  {
    if (c->GetName() == name)
    {
      c->SetRelativeLocation(GzUtil::CoordTransform(pos));
      c->SetRelativeRotation(GzUtil::CoordTransform(rot));
      return true;
    }
  }
  return false;
}


//////////////////////////////////////////////////
void FGzIface::Tick(float _delta)
{
  if (!this->GameWorld())
  {
    // if previously initialized but now we can't find game world,
    // this probably means we're playing in editor, so shut down gznode.
    if (this->dataPtr->initialized)
      this->ShutDown();
    return;
  }

  if (!this->dataPtr->initialized)
  {
    this->Init();
    this->Sync();
  }

  // process scene message
  for (auto &s : this->dataPtr->sceneMsgs)
  {
    FString sceneName = s->GetStringField("name");
    TArray<TSharedPtr<FJsonValue>> models = s->GetArrayField("model");
    for (auto m :  models)
    {
      this->CreateModelFromMsg(m->AsObject());
    }
  }
  this->dataPtr->sceneMsgs.clear();

  // process model message
  for (auto &m : this->dataPtr->modelMsgs)
  {
    FString modelName = m->GetStringField("name");
    this->CreateModelFromMsg(m);
  }
  this->dataPtr->modelMsgs.clear();

  // process pose message
  for (auto &p : this->dataPtr->poseMsgs)
  {
    FString modelName = p->GetStringField("name");
    this->UpdatePoseFromMsg(p);
  }
  this->dataPtr->poseMsgs.clear();
}

//////////////////////////////////////////////////
bool FGzIface::IsTickable() const
{
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::IsTickableInEditor() const
{
  // enable this for now - so we can detect playing and exiting in simulate
  // mode in editor
  // return false;
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::IsTickableWhenPaused() const
{
  return false;
}

//////////////////////////////////////////////////
TStatId FGzIface::GetStatId() const
{
  return TStatId();
}

//////////////////////////////////////////////////
UWorld *FGzIface::GameWorld()
{
  UWorld* world = nullptr;

  // TODO: check which macro can determine whether I am in editor
  // editor engine
  UEditorEngine* EditorEngine = Cast<UEditorEngine>(GEngine);
  if (EditorEngine)
  {
    world = EditorEngine->PlayWorld;
    if (world && world->IsValidLowLevel() && world->IsGameWorld())
    {
      return world;
    }
    else
    {
      // UE_LOG(LogTemp, Warning,
      //    TEXT("Unable to get GameWorld in EditorEngine"));
      // return nullptr;
    }
  }

  // game engine
  UGameEngine* gameEngine = Cast<UGameEngine>(GEngine);
  if (gameEngine)
  {
    world = gameEngine->GetGameWorld();
    if (world && world->IsValidLowLevel())
    {
      return world;
    }
    else
    {
      // UE_LOG(LogTemp, Warning,
      //    TEXT("Unable to get GameWorld in GameEngine"));
      return nullptr;
    }
  }
  // UE_LOG(LogTemp, Warning, TEXT("EditorEngine/GameEngine is null"));
  return nullptr;
}
