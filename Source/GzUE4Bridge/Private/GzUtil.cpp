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

#include "GzUtil.h"

#include "Json.h"
#include "JsonUtilities.h"


// Gz to UE4 scale conversion: meters to centimeters
const double UNIT_SCALE = 100.0;

//////////////////////////////////////////////////
void GzUtil::ParsePose(TSharedPtr<FJsonObject> _poseObj,
    FVector &_pos, FRotator &_rot)
{
  TSharedPtr<FJsonObject> posObj = _poseObj->GetObjectField("position");
  TSharedPtr<FJsonObject> orientObj = _poseObj->GetObjectField("orientation");

  // position
  double posX = posObj->GetNumberField("x");
  double posY = posObj->GetNumberField("y");
  double posZ = posObj->GetNumberField("z");
  _pos.Set(posX, posY, posZ);

  // orientation
  double orientX = orientObj->GetNumberField("x");
  double orientY = orientObj->GetNumberField("y");
  double orientZ = orientObj->GetNumberField("z");
  double orientW = orientObj->GetNumberField("w");
  _rot = FQuat(orientX, orientY, orientZ, orientW).Rotator();
}


//////////////////////////////////////////////////
FVector GzUtil::CoordTransform(const FVector &_pos)
{
  return FVector(_pos.X, -_pos.Y, _pos.Z) * UNIT_SCALE;
}

//////////////////////////////////////////////////
FRotator GzUtil::CoordTransform(const FRotator &_rot)
{
  // Convert rotations
  FRotator rot(FQuat(FVector::UpVector, -FMath::DegreesToRadians(_rot.Yaw)) *
      FQuat(FVector::RightVector, -FMath::DegreesToRadians(_rot.Pitch)) *
      FQuat(FVector::ForwardVector, FMath::DegreesToRadians(_rot.Roll)));

  return rot;
}


