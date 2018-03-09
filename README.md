# Installation

## Dependencies

### Gazebo

~~~
# install gazebo 7 or later versions
sudo apt install libgazebo7-dev
~~~

### GzWeb

GzWeb provides the websocket server communicating between gazebo and our GzUE4Bridge plugin.

More info: http://gazebosim.org/tutorials?tut=gzweb_install&branch=gzweb_1.4

    hg clone https://bitbucket.org/gzweb
    cd gzweb
    # we only need websocket server gzbridge this should be sufficient:
    npm run deploy

### Unreal Engine 4

https://wiki.unrealengine.com/Building_On_Linux

## Installing GzUE4Bridge

1. Create a new C++ project using Unreal Editor

1. Create a `Plugins` directory in your project

        cd path/to/your_project
        mkdir Plugins

1. clone the gzue4bridge repo into the `Plugins` directory

        cd Plugins
        hg clone https://bitbucket.org/osrf/gzue4bridge

1. clone third party websocket client header-only library, websocketpp

        mkdir -p gzue4bridge/ThirdParty
        cd gzue4bridge/ThirdParty
        git clone https://github.com/zaphoyd/websocketpp.git
        cd websocketpp
        git checkout 0.7.0

1. Go to Unreal Editor and enable the plugin. Edit > Plugins, scroll down to `Project` on left hand panel and search for `GzUE4Bridge`. Make sure `Enabled` is ticked.

1. Restart Unreal Editor for it to take effect

# Running GzUE4Bridge

1. Launch your Unreal project in Unreal Editor

        cd /path/to/UnrealEngine
        ./Engine/Binaries/Linux/UE4Editor path/to/your_project.uproject

1. Compile the project and GzUE4Bridge plugin if needed

1. In one terminal, launch gazebo, e.g.

        gzserver --verbose worlds/shapes.world

1. In another terminal, start gzweb's websocket server

        cd path/to/gzweb
        npm start

1. In Unreal Editor, hit Play (simulate mode) and you should see gazebo models (simple shapes) appear in the scene

