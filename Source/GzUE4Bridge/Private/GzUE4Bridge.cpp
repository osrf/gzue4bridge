// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "GzUE4Bridge.h"

#include "GzIface.h"


#define LOCTEXT_NAMESPACE "FGzUE4BridgeModule"

using namespace gazebo;
using namespace ue4;


void FGzUE4BridgeModule::StartupModule()
{
  // This code will execute after your module is loaded into memory; the exact
  // timing is specified in the .uplugin file per-module
  UE_LOG(LogTemp, Warning, TEXT("Starting up GzUE4Bridge"));
//  FGzIface &iface = FGzIface::Instance();
}

void FGzUE4BridgeModule::ShutdownModule()
{
  // This function may be called during shutdown to clean up your module.
  // For modules that support dynamic reloading, we call this function before
  // unloading the module.

  UE_LOG(LogTemp, Warning, TEXT("Unloading GzUE4Bridge"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FGzUE4BridgeModule, GzUE4Bridge)



