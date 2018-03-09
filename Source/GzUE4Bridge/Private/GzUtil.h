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

namespace gazebo
{
  namespace ue4
  {
    /// \brief Contains common utilility functions
    class GzUtil
    {
      /// \brief Transform a position vector from Gz (right handed) to UE4
      /// (left handed) coordinate system. Converts units from meters to centimers.
      /// \param[in] _pos Input position vector in Gz coordinate system and scale
      /// \return Output position vector in UE4 coordinate system and scale
      public: static FVector CoordTransform(const FVector &_pos);

      /// \brief Tranform rotation from Gz (right handed) to UE4 (left handed)
      /// coordinate system.
      /// \param[in] _pos Input rotation in Gz coordinate system
      /// \return Output rotation in UE4 coordinate system
      public: static FRotator CoordTransform(const FRotator &_rot);

      /// \brief Parse pose from a json message
      public: static void ParsePose(TSharedPtr<FJsonObject> _json,
            FVector &_pos, FRotator &_rot);
    };
  }
}
