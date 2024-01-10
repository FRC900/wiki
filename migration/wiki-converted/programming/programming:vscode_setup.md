If not already installed, install vs code using the instructions here : https:*code.visualstudio.com/docs/setup/linux. Be sure to install outside the container.

Run `code` to start vs code.
Hit ctrl-shift-X to bring up the extensions manager.  Search for and install the C++ and ROS extensions from microsoft. Also install the "ROS Package Variable" extension.

ctrl-shift-P to run a command.  Type Configurations UI, make sure this highlights "C/C++ Edit Configurations (UI)".  Select that, scroll down, change "C++ Standard" to c++17.  click on Advanced, scroll to "Compile Commands", put `${workspaceFolder}/../build/compile_commands.json` in there.  That should enable intellisense for C++ code.