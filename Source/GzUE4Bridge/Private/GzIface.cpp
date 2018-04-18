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

#include <cmath>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
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

const static std::string SDF_VERSION = "1.6";

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

  public: UWorld *world = nullptr;

  /// \brief A list of scene msgs.
  public: std::vector<TSharedPtr<FJsonObject>> sceneMsgs;

  /// \brief A list of model msgs.
  public: std::vector<TSharedPtr<FJsonObject>> modelMsgs;

  /// \brief A list of pose msgs.
  public: std::vector<TSharedPtr<FJsonObject>> posesMsgs;

  /// \brief A map of gazebo entity name and their id
  public: std::map<FString, unsigned int> entityIds;

  /// \brief Flag to indicate if unreal should lock step with Gazebo
  public: bool lockStep = true;

  /// \brief Flag to indicate if unreal has requested gazebo to step
  public: bool stepped = false;

  /// \brief Flag to indicate if unreal needs to publish pose to gazebo
  public: bool posePublished = false;

  /// \brief Falg to indicate if the unreal actors are paused
  public: bool paused = false;

  /// \brief Target sim time to step gazebo to. This is also unreal's current
  /// sim time.
  public: double targetSimTime = 0;

  /// \brief The latest gazebo sim time received from a pose message
  public: double poseSimTime = 0;

  /// \brief Entities in unreal that need to be created in Gazebo
  public: std::set<FString> entityToCreate;

  /// \brief Entity pose in unreal that need to be sync'ed in Gazebo
  public: std::set<FString> poseToVerify;

  /// \brief Flag to indicate if unreal has received a gazebo scene msg
  public: bool gzToUE4Scene = false;
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
  this->Shutdown();
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
bool FGzIface::GzToUE4Scene()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);
  // process scene message to get the initial scene tree
  // we should only get this message once
  bool sceneMsgReceived = !this->dataPtr->sceneMsgs.empty();
  for (auto &s : this->dataPtr->sceneMsgs)
  {
    TArray<TSharedPtr<FJsonValue>> models = s->GetArrayField("model");
    for (auto m :  models)
    {
      this->CreateModelFromMsg(m->AsObject());
    }
  }
  this->dataPtr->sceneMsgs.clear();

  return sceneMsgReceived;
}

//////////////////////////////////////////////////
bool FGzIface::UE4ToGzModelSync()
{
  // create actors in gazebo
  FString fname;
  for (TActorIterator<AActor> it(this->GameWorld()); it; ++it)
  {
    if (this->dataPtr->actors.find(it->GetName()) ==
        this->dataPtr->actors.end())
    {
      // store all existing actors
      // TODO filter out some unwanted ones?
      this->dataPtr->actors[it->GetName()] = *it;

      if (this->AdvertiseStaticMeshActor(*it))
      {
        this->dataPtr->entityToCreate.insert(it->GetName());
        continue;
      }

      if (this->AdvertiseSkeletalMeshActor(*it))
      {
        this->dataPtr->entityToCreate.insert(it->GetName());
        continue;
      }
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::GzToUE4ModelSync()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);

  // process model message
  for (auto &m : this->dataPtr->modelMsgs)
  {
    this->CreateModelFromMsg(m);
  }
  this->dataPtr->modelMsgs.clear();

  return true;
}

//////////////////////////////////////////////////
void FGzIface::UE4ToGzPoseSync()
{
  // publish new pose if gazebo is not being stepped, which means
  // unreal has just stepped
  if (this->dataPtr->lockStep && this->dataPtr->posePublished)
    return;

  // update pose
  FString fname;
  for (TActorIterator<AActor> it(this->GameWorld()); it; ++it)
  {
    // publish skeletal mesh actor pose to gazebo
    this->PublishSkeletalMeshActor(*it);
  }
  this->dataPtr->posePublished = true;
}

//////////////////////////////////////////////////
void FGzIface::GzToUE4PoseSync()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);

  // process pose message
  // TODO optimize and process only certain pose messages?
  for (auto &ps : this->dataPtr->posesMsgs)
  {
    // get pose timestamp
    TSharedPtr<FJsonObject> timeObj = ps->GetObjectField("time");
    this->dataPtr->poseSimTime = timeObj->GetNumberField("sec") +
        timeObj->GetNumberField("nsec")/1e9;

    TArray<TSharedPtr<FJsonValue>> poses = ps->GetArrayField("pose");
    for (auto p : poses)
    {
      TSharedPtr<FJsonObject> poseObj = p->AsObject();
      this->UpdatePoseFromMsg(poseObj);
    }
  }
  this->dataPtr->posesMsgs.clear();
}

//////////////////////////////////////////////////
bool FGzIface::AdvertiseStaticMeshActor(AActor *_actor)
{
  TArray<UStaticMeshComponent *> meshComps;
  _actor->GetComponents(meshComps);
  if (meshComps.Num() <= 0)
    return false;

  for (auto comp : meshComps)
  {
    UStaticMesh *mesh = comp->GetStaticMesh();
    if (!mesh)
      continue;

    FBox bbox = mesh->GetBoundingBox();
    FVector size = bbox.GetSize();
    FVector center = bbox.GetCenter();
    FVector min = bbox.Max;
    FVector max = bbox.Min;

    FVector scale = comp->GetComponentScale();
    size *= scale;
    size = size / GzUtil::GzToUE4Scale;

    // large mesh == skybox? skip
    if (size.X >= 32768 || size.Y >= 32768 || size.Z >= 32768)
      return false;

    FVector pos = comp->GetComponentLocation();
    FRotator rot = comp->GetComponentRotation();

/*    UE_LOG(LogTemp, Warning, TEXT(" min: %f, %f, %f"), min.X, min.Y, min.Z);
    UE_LOG(LogTemp, Warning, TEXT(" max: %f, %f, %f"), max.X, max.Y, max.Z);
    UE_LOG(LogTemp, Warning, TEXT(" pos: %f, %f, %f"), pos.X, pos.Y, pos.Z);
    UE_LOG(LogTemp, Warning, TEXT(" rot: %f, %f, %f"), rot.Roll, rot.Pitch, rot.Yaw);
    UE_LOG(LogTemp, Warning, TEXT(" scale: %f, %f, %f"), scale.X, scale.Y, scale.Z);
    UE_LOG(LogTemp, Warning, TEXT(" center: %f, %f, %f"), center.X, center.Y, center.Z);
*/

    center *= scale;
    center = GzUtil::UE4ToGz(center);
    pos = GzUtil::UE4ToGz(pos);
    rot = GzUtil::UE4ToGz(rot);

    TSharedPtr<FJsonObject> msg = MakeShareable(new FJsonObject);
    msg->SetStringField("name", _actor->GetName());
    msg->SetStringField("type", "sdf");

    std::stringstream geomStr;
    geomStr << "<geometry><box><size>"
            <<  size.X << " " << size.Y << " " << size.Z
            << "</size></box></geometry>";
    std::stringstream pivotStr;
    pivotStr << "<pose>"
             << center.X << " " << center.Y << " " << center.Z
             << " 0 0 0"
             << "</pose>";

    std::stringstream newModelStr;
    newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
                << "<model name='" << TCHAR_TO_UTF8(*_actor->GetName()) << "'>"
                <<   "<static>true</static>"
                <<   "<pose>"
                <<     pos.X << " " << pos.Y << " " << pos.Z << " "
                <<     FMath::DegreesToRadians(rot.Roll) << " "
                <<     FMath::DegreesToRadians(rot.Pitch) << " "
                <<     FMath::DegreesToRadians(rot.Yaw)
                <<   "</pose>"
                <<   "<link name='link'>"
                <<     pivotStr.str()
                <<     "<visual name='visual'>"
                <<       geomStr.str()
                <<     "</visual>"
                <<     "<collision name='collision'>"
                <<       geomStr.str()
                <<     "</collision>"
                <<   "</link>"
                << "</model>"
                << "</sdf>";
    msg->SetStringField("sdf", newModelStr.str().c_str());

    this->dataPtr->node->Publish("~/factory", msg);
  }
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::AdvertiseSkeletalMeshActor(AActor *_actor)
{
  TArray<USkeletalMeshComponent *> meshComps;
  _actor->GetComponents(meshComps);
  if (meshComps.Num() <= 0)
    return false;

  for (auto comp : meshComps)
  {
/*    USkeletalMesh *mesh = comp->SkeletalMesh;
    if (!mesh)
      continue;
    UE_LOG(LogTemp, Warning, TEXT(" mesh bone mirror : %d"), mesh->SkelMirrorTable.Num());

    for (int i = 0; i < mesh->SkelMirrorTable.Num(); ++i)
    {
      int idx  = mesh->SkelMirrorTable[i].SourceIndex;
      UE_LOG(LogTemp, Warning, TEXT(" source idx: %d"), idx);

      FName bName = comp->GetBoneName(i);
      UE_LOG(LogTemp, Warning, TEXT(" bName: %s"), *bName.ToString());
    }
*/

    // FSkeletalMeshRenderData renderData =
//    TArray<FBoneIndexType> activeBoneIndices =
//      comp->GetSkeletalMeshResource()->LODModels[0].ActiveBoneIndices;
//    UE_LOG(LogTemp, Warning, TEXT(" mesh bone indices: %d"),
//      static_cast<int>(boneIndices.Num()));




    FVector pos = comp->GetComponentLocation();
    FRotator rot = comp->GetComponentRotation();
    // FVector scale = comp->GetComponentScale();
    pos = GzUtil::UE4ToGz(pos);
    rot = GzUtil::UE4ToGz(rot);

//    TArray<FBoneIndexType> boneIndices = comp->RequiredBones;
//    TArray<FTransform> boneTransforms = comp->BoneSpaceTransforms;
//    TArray<FName> boneNames;
//    comp->GetBoneNames(boneNames);

//    UE_LOG(LogTemp, Warning, TEXT(" Required bones: %d"), boneIndices.Num());
//    UE_LOG(LogTemp, Warning, TEXT(" bone name size: %d"), boneNames.Num());
//    UE_LOG(LogTemp, Warning, TEXT(" bone transformss: %d"), boneTransforms.Num());

/*    for (auto n : boneTransforms)
    {
      UE_LOG(LogTemp, Warning, TEXT(" %s: %f, %f, %f, %f, %f, %f"),
      *boneNames[i].ToString(),
      n.GetTranslation().X, n.GetTranslation().Y, n.GetTranslation().Z,
      n.GetRotation().Euler().X, n.GetRotation().Euler().Y,
      n.GetRotation().Euler().Z);

      UE_LOG(LogTemp, Warning, TEXT(" vs %s: %f, %f, %f, %f, %f, %f"),
      *boneNames[i].ToString(),
      comp->GetBoneTransform(ii).GetTranslation().X,
      comp->GetBoneTransform(ii).GetTranslation().Y,
      comp->GetBoneTransform(ii).GetTranslation().Z,
      comp->GetBoneTransform(ii).GetRotation().Euler().X,
      comp->GetBoneTransform(ii).GetRotation().Euler().Y,
      comp->GetBoneTransform(ii).GetRotation().Euler().Z);

      FName boneName = comp->GetBoneName(ii);
      std::string nameStr = TCHAR_TO_UTF8(*boneName.ToString());
      FVector bonePos =
          comp->GetBoneLocation(boneName, EBoneSpaces::ComponentSpace);
      FQuat boneQuat =
          comp->GetBoneQuaternion(boneName, EBoneSpaces::ComponentSpace);

      bonePos = GzUtil::UE4ToGz(bonePos);
      FRotator boneRot = GzUtil::UE4ToGz(FRotator(boneQuat));

      UE_LOG(LogTemp, Warning, TEXT(" %s: %f, %f, %f, %f, %f, %f"),
      *boneNames[ii].ToString(),
      bonePos.X, bonePos.Y, bonePos.Z,
      boneRot.Roll, boneRot.Pitch, boneRot.Yaw);
    }
*/

    TSharedPtr<FJsonObject> msg = MakeShareable(new FJsonObject);
    msg->SetStringField("name", _actor->GetName());
    msg->SetStringField("type", "sdf");

    double jointRadius = 0.01;
    std::stringstream jointGeomStr;
    jointGeomStr << "<geometry><sphere>"
            << "<radius>"<<  jointRadius << "</radius>"
            << "</sphere></geometry>";

    std::string jointDiffuseStr = "1 1 0 1";
    std::stringstream jointVisMatStr;
    jointVisMatStr << "<material>"
            << "<diffuse>"<<  jointDiffuseStr << "</diffuse>"
            << "</material>";

    std::string boneDiffuseStr = "0 0 0.4 1";
    std::stringstream boneVisMatStr;
    boneVisMatStr << "<material>"
            << "<diffuse>"<<  boneDiffuseStr << "</diffuse>"
            << "</material>";


    std::stringstream newModelStr;
    newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
                << "<model name='" << TCHAR_TO_UTF8(*_actor->GetName()) << "'>"
                <<   "<static>true</static>"
                <<   "<pose>"
                <<      pos.X << " " << pos.Y << " " << pos.Z << " "
                <<      rot.Roll << " " << rot.Pitch << " " << rot.Yaw
                <<   "</pose>";

    std::map<FName, FVector> bonePoseMap;
//    for (int i = 0; i < comp->GetNumBones(); ++i)
    // loop through only the bones that are active in this skeletal mesh,
    // which helps to ignore bones without weight, e.g. IK bones
    TArray<FBoneIndexType> activeBoneIndices =
      comp->GetSkeletalMeshResource()->LODModels[0].ActiveBoneIndices;

    std::set<int> boneIndices;
    for (int i = 0; i < activeBoneIndices.Num();++i)
    {
      FBoneIndexType idx = activeBoneIndices[i];
//      UE_LOG(LogTemp, Warning, TEXT("   mesh index: %d"),
//        static_cast<int>(idx));
      boneIndices.insert(static_cast<int>(idx));
    }
//    for (int i = 0; i < activeBoneIndices.Num(); ++i)
    for (int i = 0; i < comp->GetNumBones(); ++i)
//    for (unsigned int i = 0; i < boneIndices.size(); ++i)
    {
//      FBoneIndexType idx = activeBoneIndices[i];
      if (boneIndices.find(i) == boneIndices.end())
        continue;

      // current bone
//      FName boneName = comp->GetBoneName(static_cast<int>(idx));
      FName boneName = comp->GetBoneName(i);
      FVector bonePos;
      auto bIt = bonePoseMap.find(boneName);
      if (bIt == bonePoseMap.end())
      {
        bonePos =
            comp->GetBoneLocation(boneName, EBoneSpaces::ComponentSpace);
        bonePos = GzUtil::UE4ToGz(bonePos);
        bonePoseMap[boneName] = bonePos;
      }
      else
      {
        bonePos = bIt->second;
      }

      FVector bonePosMid(0, 0, 0);
      FRotator boneRotMid(0, 0, 0);;
      double boneLength = 0.0;

      // parent bone
      FName boneParentName = comp->GetParentBone(boneName);
      int boneParentIdx = comp->GetBoneIndex(boneParentName);
      if (!boneParentName.IsNone() &&
          boneIndices.find(boneParentIdx) != boneIndices.end())
      {
        FVector boneParentPos;
        bIt = bonePoseMap.find(boneParentName);
        if (bIt == bonePoseMap.end())
        {
          boneParentPos =
              comp->GetBoneLocation(boneParentName, EBoneSpaces::ComponentSpace);
          boneParentPos = GzUtil::UE4ToGz(boneParentPos);
          bonePoseMap[boneParentName] = boneParentPos;
        }
        else
        {
          boneParentPos = bIt->second;
        }
        FVector dBonePos = boneParentPos - bonePos;
        boneLength = std::fabs(dBonePos.Size());
        if (!FMath::IsNearlyEqual(boneLength, 0, 1e-3))
        {
          bonePosMid = FVector(0, 0, boneLength * 0.5);
          FVector u = dBonePos.GetSafeNormal();
          FVector v(0, 0, 1);
          FQuat q = FQuat::FindBetweenVectors(v, u);
          // TODO figure out why negate
          FRotator r(q);
          r.Yaw = -r.Yaw;
          r.Roll= -r.Roll;
          boneRotMid = GzUtil::UE4ToGz(r);
        }
      }

      // create the joints and bones
      std::stringstream jointPoseStr;
      jointPoseStr << "<pose>"
                   << bonePos.X << " " << bonePos.Y << " " << bonePos.Z << " "
                   << FMath::DegreesToRadians(boneRotMid.Roll) << " "
                   << FMath::DegreesToRadians(boneRotMid.Pitch) << " "
                   << FMath::DegreesToRadians(boneRotMid.Yaw)
                   << "</pose>";

      std::stringstream bonePoseStr;
      bonePoseStr << "<pose>"
                  << bonePosMid.X << " "
                  << bonePosMid.Y << " "
                  << bonePosMid.Z << " "
                  << "0 0 0</pose>";

      std::string nameStr = TCHAR_TO_UTF8(*boneName.ToString());
      newModelStr << "<link name='" << nameStr << "'>"
                  <<   jointPoseStr.str()
                  <<   "<visual name='joint_visual'>"
                  <<     jointGeomStr.str()
                  <<     jointVisMatStr.str()
                  <<   "</visual>"
                  <<   "<collision name='joint_collision'>"
                  <<     jointGeomStr.str()
                  <<   "</collision>";

      // add bone visual only if length is > 0
      if (!FMath::IsNearlyEqual(boneLength, 0))
      {
        double boneRadius = 0.005;
        std::stringstream boneGeomStr;
        boneGeomStr << "<geometry><cylinder>"
                << "<radius>"<<  boneRadius << "</radius>"
                << "<length>"<< boneLength << "</length>"
                << "</cylinder></geometry>";

        newModelStr <<   "<visual name='bone_visual'>"
                    <<     bonePoseStr.str()
                    <<     boneGeomStr.str()
                    <<     boneVisMatStr.str()
                    <<   "</visual>"
                    <<   "<collision name='bone_collision'>"
                    <<     bonePoseStr.str()
                    <<     boneGeomStr.str()
                    <<   "</collision>";
      }
      newModelStr << "</link>";
    }
    newModelStr << "</model>"
                << "</sdf>";

    msg->SetStringField("sdf", newModelStr.str().c_str());

    this->dataPtr->node->Publish("~/factory", msg);
  }
  return true;
}

//////////////////////////////////////////////////
bool FGzIface::PublishSkeletalMeshActor(AActor *_actor)
{
  // see if unique id is available
  auto it = this->dataPtr->entityIds.find(_actor->GetName());
  if (it == this->dataPtr->entityIds.end())
    return false;
  int modelId = it->second;

  // check if it's skeletal mesh comp
  TArray<USkeletalMeshComponent *> meshComps;
  _actor->GetComponents(meshComps);
  if (meshComps.Num() <= 0)
    return false;


  for (auto comp : meshComps)
  {
    TSharedPtr<FJsonObject> msg = MakeShareable(new FJsonObject);
    msg->SetStringField("name", _actor->GetName());
    msg->SetNumberField("id", modelId);

    FVector pos = comp->GetComponentLocation();
    FRotator rot = comp->GetComponentRotation();
    // FVector scale = comp->GetComponentScale();
    pos = GzUtil::UE4ToGz(pos);
    rot = GzUtil::UE4ToGz(rot);
    FQuat quat = rot.Quaternion();

    TSharedPtr<FJsonObject> posObj = MakeShareable(new FJsonObject);
    posObj->SetNumberField("x", pos.X);
    posObj->SetNumberField("y", pos.Y);
    posObj->SetNumberField("z", pos.Z);
    msg->SetObjectField("position", posObj);

    TSharedPtr<FJsonObject> quatObj = MakeShareable(new FJsonObject);
    quatObj->SetNumberField("w", quat.W);
    quatObj->SetNumberField("x", quat.X);
    quatObj->SetNumberField("y", quat.Y);
    quatObj->SetNumberField("z", quat.Z);
    msg->SetObjectField("orientation", quatObj);

    TArray<TSharedPtr<FJsonValue>> linkArrayObj;

    std::map<FName, FVector> bonePoseMap;

    TArray<FBoneIndexType> activeBoneIndices =
      comp->GetSkeletalMeshResource()->LODModels[0].ActiveBoneIndices;
    std::set<int> boneIndices;
    for (int i = 0; i < activeBoneIndices.Num();++i)
    {
      FBoneIndexType idx = activeBoneIndices[i];
      boneIndices.insert(static_cast<int>(idx));
    }

    for (int i = 0; i < comp->GetNumBones(); ++i)
//    for (unsigned int i = 0; i < boneIndices.size(); ++i)
    {
      if (boneIndices.find(i) == boneIndices.end())
        continue;

      // current bone
      FName boneName = comp->GetBoneName(i);
//      FName boneName = comp->GetBoneName(static_cast<int>(idx));
      FString boneNameScoped = _actor->GetName() + "::" + boneName.ToString();

      // see if unique id is available
      auto lIt = this->dataPtr->entityIds.find(boneNameScoped);
      if (lIt == this->dataPtr->entityIds.end())
        continue;
      int linkId = lIt->second;

      // joint
      FVector bonePos;

      auto bIt = bonePoseMap.find(boneName);
      if (bIt == bonePoseMap.end())
      {
        bonePos =
            comp->GetBoneLocation(boneName, EBoneSpaces::ComponentSpace);
        bonePos = GzUtil::UE4ToGz(bonePos);
        bonePoseMap[boneName] = bonePos;
      }
      else
      {
        bonePos = bIt->second;
      }

      FVector bonePosMid(0, 0, 0);
      FQuat boneQuatMid(0, 0, 0, 1);;
      double boneLength = 0.0;

      // parent bone
      FName boneParentName = comp->GetParentBone(boneName);
      int boneParentIdx = comp->GetBoneIndex(boneParentName);
      if (!boneParentName.IsNone() &&
          boneIndices.find(boneParentIdx) != boneIndices.end())
      {

        FVector boneParentPos;
        FQuat boneParentQuat;

        bIt = bonePoseMap.find(boneParentName);
        if (bIt == bonePoseMap.end())
        {
          boneParentPos =
              comp->GetBoneLocation(boneParentName, EBoneSpaces::ComponentSpace);
          boneParentPos = GzUtil::UE4ToGz(boneParentPos);
          bonePoseMap[boneParentName] = boneParentPos;
        }
        else
        {
          boneParentPos = bIt->second;
        }

        FVector dBonePos = boneParentPos - bonePos;
        boneLength = std::fabs(dBonePos.Size());
        if (!FMath::IsNearlyEqual(boneLength, 0, 1e-3))
        {
          bonePosMid = FVector(0, 0, boneLength * 0.5);
          FVector u = dBonePos.GetSafeNormal();
          FVector v(0, 0, 1);
          boneQuatMid = FQuat::FindBetweenVectors(v, u);
        }
      }

      TSharedPtr< FJsonObject > linkObj = MakeShareable(new FJsonObject);
      linkObj->SetStringField("name", boneNameScoped);
      linkObj->SetNumberField("id", linkId);

      TSharedPtr< FJsonObject > linkPosObj = MakeShareable(new FJsonObject);
      linkPosObj->SetNumberField("x", bonePos.X);
      linkPosObj->SetNumberField("y", bonePos.Y);
      linkPosObj->SetNumberField("z", bonePos.Z);
      TSharedPtr< FJsonObject > linkQuatObj = MakeShareable(new FJsonObject);
      linkQuatObj->SetNumberField("w", boneQuatMid.W);
      linkQuatObj->SetNumberField("x", boneQuatMid.X);
      linkQuatObj->SetNumberField("y", boneQuatMid.Y);
      linkQuatObj->SetNumberField("z", boneQuatMid.Z);
      linkObj->SetObjectField("position", linkPosObj);
      linkObj->SetObjectField("orientation", linkQuatObj);

      // UE_LOG(LogTemp, Warning, TEXT("=== bone pose: %f %f %f,  %f %f %f %f"),
      //  bonePos.X, bonePos.Y, bonePos.Z, boneQuat.W, boneQuat.X, boneQuat.Y, boneQuat.Z);

      TSharedRef<FJsonValueObject> linkValue =
          MakeShareable(new FJsonValueObject(linkObj));
      linkArrayObj.Add(linkValue);
    }
    msg->SetArrayField("link", linkArrayObj);

    this->dataPtr->node->Publish("~/model/modify", msg);

    // remember the model name so we can verify its pose has been received by
    // gazebo
    this->dataPtr->poseToVerify.insert(_actor->GetName());
  }

  return true;
}

//////////////////////////////////////////////////
bool FGzIface::StepGz(const int _steps)
{
  TSharedPtr<FJsonObject> msg = MakeShareable(new FJsonObject);
  msg->SetBoolField("pause", true);
  msg->SetNumberField("multi_step", _steps);

  this->dataPtr->node->Publish("~/world_control", msg);
  return true;
}

//////////////////////////////////////////////////
void FGzIface::SetActorsPaused(const bool _paused)
{
  if (_paused == this->dataPtr->paused)
    return;

  this->dataPtr->paused = _paused;

  double timeDilation = _paused ? 0 : 1;

  FString fname;
  for (TActorIterator<AActor> it(this->GameWorld()); it; ++it)
  {
//    if (this->dataPtr->actors.find(it->GetName()) !=
//        this->dataPtr->actors.end())
    {
      (*it)->CustomTimeDilation = timeDilation;
    }
  }
}

//////////////////////////////////////////////////
void FGzIface::Shutdown()
{
  delete this->dataPtr->node;
  this->dataPtr->node = nullptr;
  this->dataPtr->initialized = false;
  this->dataPtr->targetSimTime = 0;
  this->dataPtr->poseSimTime = 0;
  this->dataPtr->paused = false;
  this->dataPtr->stepped = false;
  this->dataPtr->posePublished = false;
  this->dataPtr->gzToUE4Scene = false;
  this->dataPtr->world = nullptr;

  this->dataPtr->actors.clear();
  this->dataPtr->entityIds.clear();
  this->dataPtr->modelMsgs.clear();
  this->dataPtr->posesMsgs.clear();
  this->dataPtr->sceneMsgs.clear();
  this->dataPtr->entityToCreate.clear();
  this->dataPtr->poseToVerify.clear();
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
  this->dataPtr->posesMsgs.push_back(_json);
}

//////////////////////////////////////////////////
bool FGzIface::CreateModelFromMsg(TSharedPtr<FJsonObject> _json)
{
  FString modelName = _json->GetStringField("name");

  // if a gazebo model does not exist in UE, add it to the actors map
  if (this->dataPtr->actors.find(modelName) == this->dataPtr->actors.end())
  {
    FVector pos(0, 0, 0);
    FRotator rot(0, 0, 0);
    FActorSpawnParameters spawnParams;
    spawnParams.Name = FName(*modelName);
    AGzModel* model =
        this->GameWorld()->SpawnActor<AGzModel>(pos, rot, spawnParams);
    model->Load(_json);
    model->CustomTimeDilation = this->dataPtr->lockStep ? 0 : 1;
    this->dataPtr->actors[model->GetName()] = model;
  }

  // update entity id map
  if (this->dataPtr->entityIds.find(modelName) ==
      this->dataPtr->entityIds.end())
  {
    unsigned int modelId = _json->GetIntegerField("id");
    this->dataPtr->entityIds[modelName] = modelId;
    TArray<TSharedPtr<FJsonValue>> links = _json->GetArrayField("link");
    for (auto link : links)
    {
      TSharedPtr<FJsonObject> linkObj = link->AsObject();
      unsigned int linkId = linkObj->GetIntegerField("id");
      FString linkName = linkObj->GetStringField("name");
      this->dataPtr->entityIds[linkName] = linkId;
    }
  }

  // remove entry in entity to create so that we know it's been created in
  // gazebo.
  auto entIt = this->dataPtr->entityToCreate.find(modelName);
  if (entIt != this->dataPtr->entityToCreate.end())
  {
    this->dataPtr->entityToCreate.erase(entIt);
  }

  // remove entry in pose to create set so that we know it's been updated in
  // gazebo.
  auto poseIt = this->dataPtr->poseToVerify.find(modelName);
  if (poseIt != this->dataPtr->poseToVerify.end())
  {
    this->dataPtr->poseToVerify.erase(poseIt);
  }

  return true;
}

//////////////////////////////////////////////////
bool FGzIface::UpdatePoseFromMsg(TSharedPtr<FJsonObject> _json)
{
  if (!_json.IsValid())
    return false;


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

  // ignore skeletal pose updates from gz as
  TArray<USkeletalMeshComponent *> meshComps;
  it->second->GetComponents(meshComps);
  if (meshComps.Num() > 0)
    return false;

  FVector pos;
  FRotator rot;
  GzUtil::ParsePose(_json, pos, rot);

  if (isModel)
  {
    it->second->SetActorLocation(GzUtil::GzToUE4(pos));
    it->second->SetActorRotation(GzUtil::GzToUE4(rot));
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
      c->SetRelativeLocation(GzUtil::GzToUE4(pos));
      c->SetRelativeRotation(GzUtil::GzToUE4(rot));

      UE_LOG(LogTemp, Warning, TEXT("  update pose from msg done "));
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
void FGzIface::Tick(float _delta)
{
  // Get World
  if (!this->GameWorld())
  {
    if (this->dataPtr->initialized)
      this->Shutdown();
    return;
  }

  // Init by subscribing to Gz topics
  if (!this->dataPtr->initialized)
  {
    this->Init();
  }

  // pause the actors if we're lockstepping
  if (this->dataPtr->lockStep)
    this->SetActorsPaused(true);

  ////////////////////
  // Sync Scene
  ////////////////////

  // Reconstruct gazebo scene in UE4
  if (!this->dataPtr->gzToUE4Scene)
  {
    if (this->GzToUE4Scene())
      this->dataPtr->gzToUE4Scene = true;
    else
      return;
  }

  ////////////////////
  // Sync Models
  ////////////////////

  // Create UE4 models in gazebo
  this->UE4ToGzModelSync();

  // Create gazebo models in UE4
  this->GzToUE4ModelSync();

  // Wait and verify UE4 models are created in gazebo before stepping.
  if (this->dataPtr->lockStep && !this->dataPtr->entityToCreate.empty())
    return;


  /// TODO remove me
  // return;

  ////////////////////
  // Sync Pose
  ////////////////////

  // Sync UE4 model pose to Gz
  this->UE4ToGzPoseSync();

  // Sync Gz model pose to UE4
  this->GzToUE4PoseSync();

  // Wait and verify UE4 model poses are updated in gazebo before stepping.
  if (this->dataPtr->lockStep && !this->dataPtr->poseToVerify.empty())
    return;

  ////////////////////
  // Step
  ////////////////////

  // lock step
  if (this->dataPtr->lockStep)
  {
    // first UE4 step
    if (this->dataPtr->targetSimTime <= 0)
    {
      this->dataPtr->targetSimTime = this->GameWorld()->GetDeltaSeconds();
    }

    // UE4 step size
    double delta = this->GameWorld()->GetDeltaSeconds();

    // if UE4 is ahead
    if (this->dataPtr->poseSimTime < this->dataPtr->targetSimTime)
    {
      // check if we need to step gazebo
      if (!this->dataPtr->stepped)
      {
        // IMPORTANT: Set UE4 to use a fixed frame rate that is a factor of
        // Gazebo's physics update rate (1000Hz), e.g. If UE4 is set to run at
        // 50Hz (at fixed step size of 20ms) then the number of steps for Gazebo
        // to take will be:
        //   1000 / 50 = 20 steps
        // Settings are in Edit > Project Settings > Engine > General Settings.
        // Make sure the 'Use Fixed Frame Rate' checkbox is checked and specify
        // a Fixed Frame Rate that is a factor of 1000
        int gzRate = 1000;
        int steps = static_cast<int>(std::round(gzRate * delta));
        this->StepGz(steps);
        this->dataPtr->stepped = true;
      }
    }
    // if gazebo caught up, we're in sync, now step UE4
    else
    {
      this->dataPtr->targetSimTime =
          this->dataPtr->poseSimTime + delta;

      this->SetActorsPaused(false);
      this->dataPtr->stepped = false;
      this->dataPtr->posePublished = false;
    }
  }
}

//////////////////////////////////////////////////
UWorld *FGzIface::GameWorld()
{
  if (this->dataPtr->world)
    return this->dataPtr->world;

  UWorld* world = nullptr;

  // TODO: check which macro can determine whether I am in editor
  // editor engine
  UEditorEngine* EditorEngine = Cast<UEditorEngine>(GEngine);
  if (EditorEngine)
  {
    world = EditorEngine->PlayWorld;
    if (world && world->IsValidLowLevel() && world->IsGameWorld())
    {
      this->dataPtr->world = world;
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
      this->dataPtr->world = world;
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
