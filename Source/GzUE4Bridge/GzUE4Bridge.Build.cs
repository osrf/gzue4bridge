// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class GzUE4Bridge : ModuleRules
{
  private string ModulePath
  {
    get { return ModuleDirectory; }
  }

  private string ThirdPartyPath
  {
    get { return Path.GetFullPath( Path.Combine( ModulePath, "../../ThirdParty/" ) ); }
  }

	public GzUE4Bridge(ReadOnlyTargetRules Target) : base(Target)
	{
//		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
/*		PublicIncludePaths.AddRange(
			new string[] {
				"GzUE4Bridge/Public"
				// ... add public include paths required here ...
			}
			);
*/


		PrivateIncludePaths.AddRange(
			new string[] {
				"GzUE4Bridge/Private",
				// ... add other private include paths required here ...
			}
			);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
        "Core",
        "CoreUObject",
        "Engine",
				// ... add other public dependencies that you statically link with here ...
        "Json",
        "JsonUtilities",
			}
			);

/*
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				// ... add private dependencies that you statically link with here ...
 			}
			);
*/


/*		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
*/

    //////////////////////
    if (Target.Platform == UnrealTargetPlatform.Linux)
    {
      // bUseRTTI = true;
      PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

      PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "websocketpp"));
      string prefix = "/usr";
      string includePrefix = Path.Combine(prefix, "include");
      string libPrefix = Path.Combine(prefix, "lib", "x86_64-linux-gnu");
      PublicAdditionalLibraries.Add(Path.Combine(libPrefix, "libboost_system.so"));
    }

	}
}
