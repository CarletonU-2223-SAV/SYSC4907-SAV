### Simulated Autonomous Vehicle Engine

### Setting up the Environment

Windows was used for development. Using WSL is possible, but it is harder to set up. At time of writing, AirSim does not have
executables for MacOS and only some for Linux.

1. Install ROS noetic for windows. Follow the instructions here: http://wiki.ros.org/noetic/Installation/Windows

    > Visual Studio Community 2019 was used
    > 
    > When following step 5, only step 5.1 is needed
    > 
    > When following step 6, only the command for the Community edition of Visual Studio was used. Additionally, not setting the created shortcut with admin rights still worked, and shortcut can be created by right-clicking the Windows desktop (New -> shortcut)
    > 
    > Compatibility with Visual Studio 2022 was tested and the project was installed successfully. When using Visual Studio 2022, the path to the executable is slightly different. By default, it is installed under C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat
    > 
    > Steps 6.1, 7, 8, 9 were not followed, but worthwhile to keep in mind

2. Download an AirSim release. These can be found here (scroll to a Windows release section): https://github.com/Microsoft/AirSim/releases . Development was done for the neighbourhood release (AirSimNH) and to a lesser extent the coastal environment. The code cloned without modifications works by default for neighbourhood. 

   >Version 1.6 was used, but restrictions for using other versions are not known
   >
   >7z is recommended for extracting if the scene consists of multiple zipped files (read "Downloading large environments" in a release section)
   
    >After extracting a scene and trying to launch its executable, an error about not having DirectX installed may appear. In this case, DirectX runtime may have to be installed - this can be found here: https://www.microsoft.com/en-ca/download/confirmation.aspx?id=35
    > * Extra important: If installing DirectX, read each installation screen carefully, and make sure to uncheck "Install the Bing Bar"- you're better than that

3. ***Optional*** Install Pycharm IDE - this was used during development. Any environment that allows Python to be written works

### First Usage

1. Open up a ROS console, and navigate to the folder with the cloned repository.
   > Execute: catkin_make

   > Certain dependencies are needed. To get all of them, execute: pip install --no-deps -r dependencies.txt . Before performing this command, pip may need to be updated (using "pip with python -m pip install --upgrade pip") followed by installing msgpack with "pip install msgpack-rpc-python"
   
    * Follow next step "Subsequent Usage"

### Subsequent Usage

1. Launch Airsim. A popup window may appear asking if a Car Simulation should be used. Click Yes.
    > In the top left corner, a line will say "Loaded settings from ...". (Note that for the first time Airsim is launched, this line may not appear. Relaunch Airsim in this case). This file is equivalent to the settings.json file in this repository. 
     Make sure that that file mentioned in AirSim is the same as the settings file in this repository.
    >
    > To run the simulation at a specific window simulation in windowed mode, the command line can be used.
      In the terminal, navigate to the AirSim executable and invoke it wth the following flags:  -ResX=width -ResY=height -windowed
    >
    > For example: AirsimNH -ResX=640 -ResY=480 -windowed 

2. Open up a ROS console, and navigate to the folder with the cloned repository.
    > ***(Any time a new ROS node is added)*** Execute: catkin_make
    >
    > ***(Required everytime ROS console is opened)*** Execute: .\devel\setup.bat
    >
    > Execute: roslaunch central_control full_system.launch
    > 
    > There is a chance of getting this message in the ROS console: 31mRLException: [full_system.launch] is neither a launch file in package [central_control] nor is [central_control] a launch file name.
      To fix this, make sure the cloned repository's filepath does not contain any whitespace, then restart this step. The build and devel folders may need to be deleted first.
     
### Other Instructions

1. Exit Ros with ctrl+c in the ROS console. Do this before closing Airsim, which should be relaunched for every test run to start from a known situation in Airsim.

2. To create a new test route, download a standard installation of Python for the desktop (ROS' installation of it does not come with Tkinter). In a command prompt, navigate to the cloned project directory, go to src/mapping_navigation/scripts and execute "python TestGUI.py" to launch the route making GUI. The command "pip install Pillow" may need to be done first.

    > In the GUI, click "Create" followed by "straight road". Then click the new "RoadSegmentType.STRAIGHT" label to the right of the map. Then click "Create" follwed by "lane". Then click the new "[]" label under the previously selected RoadSegementType label. Then click "Create" then "path".  Now start clicking points on the map to add points. Repeat these steps for intersections and other paths that do not involve intersections.
    > 
    > Afterwards, click the first "empty path" label to the right of the map. Then click all of the created points on the map. 
    > 
    > Create an empty path file somewhere on the file system, ending with the ".pickle" file extension. Then click "File", "save map", and select the created pickle file.
    > 
    > Finally, go to the mapping_navigation/src/scripts/navigation.py file, go to the main function and set the map variable to the location of the created pickle file
