# Installation


### Gazebo

~~~
# install gazebo 7+
sudo apt install libgazebo7-dev
~~~

### GzWeb

GzWeb provides a websocket server for communicating between gazebo and the GzUE4Bridge plugin in Unreal.

More info: http://gazebosim.org/tutorials?tut=gzweb_install&branch=gzweb_1.4

    hg clone https://bitbucket.org/gzweb
    cd gzweb
    hg up pose_msgs
    # we only need websocket server gzbridge this should be sufficient:
    npm run deploy

### Unreal Engine 4

https://wiki.unrealengine.com/Building_On_Linux

## Installing GzUE4Bridge Plugin

1. Create a new C++ project using Unreal Editor

1. Create a `Plugins` directory in your project

        cd path/to/your_project
        mkdir Plugins

1. Clone the `gzue4bridge` repo into the `Plugins` directory

        cd Plugins
        hg clone https://bitbucket.org/osrf/gzue4bridge

1. Clone third party websocket client library (header-only), `websocketpp`

        mkdir -p gzue4bridge/ThirdParty
        cd gzue4bridge/ThirdParty
        git clone https://github.com/zaphoyd/websocketpp.git
        cd websocketpp
        git checkout 0.7.0

1. Go to Unreal Editor and enable the plugin. `Edit` > `Plugins`, scroll down to `Project` on left hand panel and search for `GzUE4Bridge`. Make sure `Enabled` is ticked.

1. Restart Unreal Editor for it to take effect

    Note: You may need to edit `gzue4bridge/Source/GzUE4Bridge/GzUE4Bridge.Build.cs` and update the path to boost library. TODO: need to figure out a cleaner way to do this.

# Configuring Unreal project for lock-stepping with Gazebo

1. Launch your Unreal project in Unreal Editor

        cd /path/to/UnrealEngine
        ./Engine/Binaries/Linux/UE4Editor path/to/your_project.uproject

1. Compile the project and GzUE4Bridge plugin if needed

1. Set Unreal to use fixed time step and a framerate that is a factor of Gazebo's physics update rate

    Edit > Project Settings > Engine > General Settings > Framerate: Make sure `Use Fixed Frame Rate` is checked and set `Fixed Frame Rate` to 50

1. Set your project to use GzUE4Bridge's game instance.

    Edit > Project Settings > Project > Maps & Modes > Game Instance: In the drop down list next to `Game Instance Class` select `GzIfaceGameInstance`

# Running GzUE4Bridge

1. In one terminal, launch gazebo server in paused mode, e.g.

        gzserver --verbose -u

1. In another terminal, start gzweb's websocket server with pose filter disabled

        cd path/to/gzweb
        npm start -- -a

    you should see the message:

        Gazebo transport node connected to gzserver.
        Pose message filter disabled. Messages are published at full rate.


1. In Unreal Editor, hit the Play button (simulate mode) to start simulation.

