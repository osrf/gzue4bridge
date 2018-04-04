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

#include "GzModel.h"

#include "Json.h"
#include "JsonUtilities.h"

//#include "ConstructorHelpers.h"
#include <Runtime/Engine/Classes/Components/SphereComponent.h>
#include <Runtime/Engine/Classes/Components/BoxComponent.h>
//#include <Runtime/CoreUObject/Public/UObject/UObjectGlobals.h>
//#include <Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h>

#include "GzUtil.h"

// Sets default values
AGzModel::AGzModel(const FObjectInitializer& ObjectInitializer)
{
  // Set this actor to call Tick() every frame.
  // You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
}

//////////////////////////////////////////////////
void AGzModel::Load(TSharedPtr<FJsonObject> _json)
{
  FString name = _json->GetStringField("name");
  USceneComponent *modelComp =
      NewObject<USceneComponent>(this, USceneComponent::StaticClass(),
      FName(*name));
  modelComp->RegisterComponent();
  modelComp->bHiddenInGame = false;
  modelComp->Mobility = EComponentMobility::Movable;
  RootComponent = modelComp;

  if (_json->HasField("link"))
  {
    TArray<TSharedPtr<FJsonValue>> links = _json->GetArrayField("link");
    for (auto l : links)
    {
      this->LoadLink(l->AsObject(), modelComp);
    }
  }

  // pose
  if (_json->HasField("pose"))
  {
    TSharedPtr<FJsonObject> poseObj = _json->GetObjectField("pose");
    FVector pos;
    FRotator rot;
    GzUtil::ParsePose(poseObj, pos, rot);
    SetActorRotation(GzUtil::GzToUE4(rot));
    SetActorLocation(GzUtil::GzToUE4(pos));
  }
}

//////////////////////////////////////////////////
void AGzModel::LoadLink(TSharedPtr<FJsonObject> _json, USceneComponent *_parent)
{
  // name
  FString name = _json->GetStringField("name");
  USceneComponent *linkComp =
      NewObject<USceneComponent>(_parent, USceneComponent::StaticClass(),
      FName(*name));
  linkComp->RegisterComponent();
  linkComp->bHiddenInGame = false;
  linkComp->Mobility = EComponentMobility::Movable;
  linkComp->AttachToComponent(_parent,
      FAttachmentTransformRules::KeepRelativeTransform);

  // pose
  if (_json->HasField("pose"))
  {
    TSharedPtr<FJsonObject> poseObj = _json->GetObjectField("pose");
    FVector pos;
    FRotator rot;
    GzUtil::ParsePose(poseObj, pos, rot);
    linkComp->SetRelativeLocation(GzUtil::GzToUE4(pos));
    linkComp->SetRelativeRotation(GzUtil::GzToUE4(rot));
  }

  // visual
  if (_json->HasField("visual"))
  {
    TArray<TSharedPtr<FJsonValue>> visuals = _json->GetArrayField("visual");
    for (auto v : visuals)
    {
      this->LoadVisual(v->AsObject(), linkComp);
    }
  }
}

//////////////////////////////////////////////////
void AGzModel::LoadVisual(TSharedPtr<FJsonObject> _json,
    USceneComponent *_parent)
{
  // name
  FString name = _json->GetStringField("name");
  USceneComponent *visualComp =
      NewObject<USceneComponent>(_parent, USceneComponent::StaticClass(),
      FName(*name));
  visualComp->RegisterComponent();
  visualComp->bHiddenInGame = false;
  visualComp->Mobility = EComponentMobility::Movable;
  visualComp->AttachToComponent(_parent,
      FAttachmentTransformRules::KeepRelativeTransform);

  // pose
  if (_json->HasField("pose"))
  {
    TSharedPtr<FJsonObject> poseObj = _json->GetObjectField("pose");
    FVector pos;
    FRotator rot;
    GzUtil::ParsePose(poseObj, pos, rot);
    visualComp->SetRelativeLocation(GzUtil::GzToUE4(pos));
    visualComp->SetRelativeRotation(GzUtil::GzToUE4(rot));
  }

  if (_json->HasField("geometry"))
  {
    TSharedPtr<FJsonObject> geomObj = _json->GetObjectField("geometry");
    this->LoadGeometry(geomObj, visualComp);
  }
}

//////////////////////////////////////////////////
void AGzModel::LoadGeometry(TSharedPtr<FJsonObject> _json,
    USceneComponent *_parent)
{
  FVector pos(0, 0, 0);
  FVector scale(1, 1, 1);
  std::string geomAssetStr;
  if (_json->HasField("box"))
  {
    geomAssetStr = "/Game/StarterContent/Shapes/Shape_Cube.Shape_Cube";
    TSharedPtr<FJsonObject> boxObj = _json->GetObjectField("box");
    if (boxObj->HasField("size"))
    {
      TSharedPtr<FJsonObject> sizeObj = boxObj->GetObjectField("size");
      double sizeX = sizeObj->GetNumberField("x");
      double sizeY = sizeObj->GetNumberField("y");
      double sizeZ = sizeObj->GetNumberField("z");
      scale.Set(sizeX, sizeY, sizeZ);
    }
    pos.Set(0, 0, -0.5*scale.Z);
  }
  else if (_json->HasField("cylinder"))
  {
    geomAssetStr = "/Game/StarterContent/Shapes/Shape_Cylinder.Shape_Cylinder";
    TSharedPtr<FJsonObject> cylinderObj = _json->GetObjectField("cylinder");
    if (cylinderObj->HasField("radius"))
    {
      double radius = cylinderObj->GetNumberField("radius");
      scale.X = radius * 2;
      scale.Y = scale.X;
    }
    if (cylinderObj->HasField("length"))
    {
      double length = cylinderObj->GetNumberField("length");
      scale.Z = length;
    }
    pos.Set(0, 0, -0.5*scale.Z);
  }
  else if (_json->HasField("sphere"))
  {
    geomAssetStr = "/Game/StarterContent/Shapes/Shape_Sphere.Shape_Sphere";
    TSharedPtr<FJsonObject> sphereObj = _json->GetObjectField("sphere");
    if (sphereObj->HasField("radius"))
    {
      double radius = sphereObj->GetNumberField("radius");
      scale.X = radius * 2;
      scale.Y = scale.X;
      scale.Z = scale.X;
    }
    pos.Set(0, 0, -0.5*scale.Z);
  }
  else
  {
    UE_LOG(LogTemp, Warning, TEXT("Visual[%s] geometry not supported yet"),
        *_parent->GetFName().ToString());
    return;
  }

  UStaticMeshComponent *geomComp =
    NewObject<UStaticMeshComponent>(_parent,
    UStaticMeshComponent::StaticClass());
  auto mesh = (UStaticMesh *)
      StaticLoadObject(UStaticMesh::StaticClass(),
      _parent, *FString(geomAssetStr.c_str()));
  geomComp->SetStaticMesh(mesh);

  geomComp->RegisterComponent();
  geomComp->bHiddenInGame = false;
  geomComp->Mobility = EComponentMobility::Movable;
  geomComp->AttachToComponent(_parent,
      FAttachmentTransformRules::KeepRelativeTransform);
  geomComp->SetRelativeLocation(GzUtil::GzToUE4(pos));
  geomComp->SetRelativeScale3D(scale);
}

// Called when the game starts or when spawned
void AGzModel::BeginPlay()
{
  Super::BeginPlay();
}

// Called every frame
void AGzModel::Tick(float DeltaTime)
{
  Super::Tick(DeltaTime);
}

