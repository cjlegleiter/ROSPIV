function addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play)
%% addRosNode.m: Create a new ROS node from MATLAB source code and add the new node to an existing ROS package
%
%% addRosNode.m:
%   Function to generate a ROS node from MATLAB source code on Windows or a 
%   virtual machine (VM). An existing MATLAB function is run through the
%   codegen process and the file management of the resulting output is handled
%   programatically by this function, including adding the new node to an
%   existing ROS package. If specified, a *.yaml file with ROS parameters will
%   be copied to the package directory and a corresponding *.launch file created
%   to load the file and populate the ROS parameter tree. An existing bag file
%   can also be provided as input so that the node can be tested. The user is
%   prompted on how to proceed via instructions printed in the MATLAB command
%   window.
%
%% SYNTAX:
%   addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play)
%
%% INPUTS:
%   matFuncDir: String specifying directory with MATLAB source code to be built
%               into a ROS node
%   packName:   String specifying name of the existing ROS package within which
%               the ROS node will be placed and called from
%   funcName:   String specifying the name of the MATLAB function to be built
%               into a ROS node
%   paramFile:  String specifying the name of a *.yaml file with ROS parameters,
%               or an empty array if no such file is needed or available
%   testFlag:   Logical flag to test the new node on the specified computing
%               platform using an existing bag file: 1 to test, 0 to just build
%   platform:   String specifying which platform to use for development-must be:
%               "win" for local machine Windows-based ROS implementation
%               "vm" for  virtual machine Linux-based ROS implementation
%   bag2play:   String specifying name of a ROS bag file to play to test node;
%               pass an empty array to just build the node and not try to test
%
%% OUTPUT:
%   A new ROS node on the selected platform callable as:
%       rosrun packName funcName
%
%% NOTES:
%   > MATLAB Coder and ROS Toolboxes required for codegen
%   > bag file must be transferred to the VM separately
%   > Tested using VM provided by Massimo
%   > Assumes that the folder C:\catkin_ws exists and already has the package
%     given by packName, this function just adds new nodes to this package
%   > Commented block of code shows how to set up the package in the first place
%   > Use the function pivEntry.m to create the PivParams.yaml file
%
%% FUNCTION SUMMARY:
%   addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play)

%% CREDITS
% Dr. Carl J. Legleiter, cjl@usgs.gov            
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey                
% With assistance from Michael Dille of NASA
% 12/16/2022
% 1/29/2023 - Updated build structure strategy based on revised pipeline from Michael Dille
% 5/15/2023 - Added ability to copy a *.yaml file with ROS parameters to the
%             package directory and create a corresponding *.launch file to load
%             the *.yaml file and populat the ROS parameter server 
% 9/6/2023 -  Clean up for new ROSPIV branch
% ~\TRiVIA\ROSPIV\addRosNode.m


%% OVERVIEW: 
% The goal is to have a single ROS package called streamflow that can contain
% several nodes. Each node should have its own set of topics it subscribes to
% for inputs and then a separate set of topics that it publishes outputs to.

% Some useful resources with guidance on ROS packages:
    % https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Creating-a-ROS-Package-and-Node.html
    % http://wiki.ros.org/ROS/Tutorials - read the first six beginner tutorials
    % http://wiki.ros.org/catkin/package.xml


%% 5/8/2023: Updated function inputs for testing with SHIVER'ed bag file
% % First we need to add the folder with the PIV code to the MATLAB search path.
% % Note that using genpath will include the subfolders as well
% pivCodePath =   genpath(fullfile(getenv("OneDrive"),"CJLmfiles\TRiVIA\RiOS\MATLAB\PIVcode"));
% addpath(pivCodePath)
% matFuncDir  =   fullfile(getenv("OneDrive"),"\CJLmfiles\TRiVIA\RiOS\MATLAB");
% packName    =   "streamflowpiv";
% funcName    =   'rosPivEntry';
% paramFile   =   "PivParams.yaml";
% testFlag    =   1;
% platform    =   "win";
% bag2play    =   fullfile(getenv("OneDrive"),"\CJLmfiles\TRiVIA\RiOS\MATLAB","SHIVER.bag");


%% Create catkin workspace and an initially empty package
wsDir   =   "C:"+filesep+"catkin_ws";
if ~isfolder(wsDir+filesep+"src")
    mkdir(wsDir+filesep+"src")
end
cd(wsDir+filesep+"src");
% Set package name as an input to this function above
% Establish package directory 
pkgDir  =   wsDir+filesep+"src"+filesep+packName;
if ~isfolder(pkgDir)
    mkdir(pkgDir)
end
% Not sure we really need this next bit, so leave out for now
    % Copy the modified CMakeLists.txt file from Michael to the package directory
    % pkgCmakeFile    =   fullfile(getenv("O"),"CJLmfiles\TRiVIA\RiOS\MATLAB\CMakeLists.txt");
    % copyfile(pkgCmakeFile,pkgDir)


%% Confirm catkin workspace with the specified package actually exists
if ~isfolder(pkgDir)
    error("The package " + packName + " does not exist in " + wsDir)
end
% Enter this directory
cd(wsDir+filesep+"src");


%% Confirm presence of package-specific CMakeLists.txt file, or copy from template
% Note that Michael provided this customized CMakeLists.txt file, which we
% should NOT have to modify as new nodes are added
% Note that the package name streamflowpiv is hard-wired into this file and will
% thus have to be changed if we change the package name at some point
pkgCmakeFile    =   "CMakeLists.txt";
if exist(fullfile(pkgDir,pkgCmakeFile),"file") == 0
    templateFile=   fullfile(getenv("OneDrive"),"CJLmfiles\TRiVIA\RiOS\MATLAB\CMakeLists.txt");
    if exist(templateFile,"file")
        copyfile(templateFile,pkgDir)
    else
        error("The CMakeLists.txt file for " + packName + " does not exist.")
    end
end


%% In MATLAB, prepare to codegen the node in a temporary folder within catkin_ws
% Set some environment variables we need to allow communication with ROS core
setenv("ROS_HOSTNAME","localhost");
setenv("ROS_MASTER_URI",'http://localhost:11311');
setenv('ROS_IP','127.0.0.1');
% We don't necessarily need to do the codegen in catkin_ws, so copy the MATLAB
% function to a temporary folder to make things less confusing
tmpDir  =   wsDir + filesep + "tmp" + filesep + funcName;
if ~isfolder(tmpDir)
    mkdir(tmpDir)
end
copyfile(fullfile(matFuncDir,funcName+".m"),tmpDir)


%% Now codegen in temporary directory to make the MATLAB function into C++ code
cd(tmpDir)
cfg                                 =   coder.config('exe');
cfg.Hardware                        =   coder.hardware('Robot Operating System (ROS)');
% ***** 12/23/2022: ATTEMPT TO RESOLVE OPEN CV COMPILATION ISSUE
    % Added the following line based on guidance from MATLAB tech support on
    % openCV compilation issue:
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';  
% ***** RESUME
cfg.Hardware.BuildAction            =   'None';
cfg.Hardware.PackageMaintainerEmail =   'cjl@usgs.gov';
cfg.Hardware.PackageMaintainerName  =   'Carl J. Legleiter';
disp("Generating ROS node from MATLAB source code for " + funcName + ".m ...")
eval(['codegen ' char(funcName) ' -args {} -config cfg'])
% Don't be alarmed by a warning about an infinite loop, we just have to CTRL+C
% to kill the node when done


%% Some reorganization to place the codegen'ed function/node within the package
% Following MATLAB's codegen, [funcName] is a folder in catkin_ws/tmp, but we 
% need to reorganize so that it will be a node within [packName], so ...
cd(tmpDir)
% First for src
copyfile(tmpDir+filesep+"src"+filesep+lower(funcName)+filesep+"src",...
         pkgDir+filesep+"src"+filesep+funcName)
% Now for include
copyfile(tmpDir+filesep+"src"+filesep+lower(funcName)+filesep+"include"+filesep+lower(funcName),...
         pkgDir+filesep+"src"+filesep+funcName)


%% **** 1/29/2023: New code to "manually" edit files to remove misleading include statements
% For now (1/29/2023), there's only one file we need to modify
codeDir     =   pkgDir+filesep+"src"+filesep+funcName;
cppFiles    =   dir(codeDir+filesep+"mwfreak.cpp"); % This could be expanded ...
allFiles    =   cppFiles;
% Specify the pattern we want to find
includePat  =   pattern("#include ""include/");
replaceWith =   "#include """;
for ii = 1:length(allFiles)
    tmp0    =   readlines(fullfile(allFiles(ii).folder,allFiles(ii).name));
    tmp1    =   replace(tmp0,includePat,replaceWith);
    writelines(tmp1,fullfile(allFiles(ii).folder,allFiles(ii).name),"WriteMode","overwrite");
end


%% **** 1/29/2023: New code to also make a lib folder - not needed?
% "From that zip, I also took src/pivcore/src/*.{lib,dll} and put those in a new
% folder src/streamflowpiv/lib/"
    % copyfile(tmpDir+filesep+"src"+filesep+lower(funcName)+filesep+"src"+filesep+"*.dll",...
    %          pkgDir+filesep+"lib")
    % copyfile(tmpDir+filesep+"src"+filesep+lower(funcName)+filesep+"src"+filesep+"*.dll",...
    %          pkgDir+filesep+"lib")
% Didn't run because there aren't any .dll or .lib files in this folder


%% If we don't already have one in pkgDir, move and edit package.xml file
if exist(fullfile(pkgDir,"package.xml"),"file") == 0
    copyfile(tmpDir+filesep+"src"+filesep+lower(funcName)+filesep+"package.xml",pkgDir)
    cd(pkgDir)
    pkgVer              =   "1.0.0";
    pkgDesc             =   "streamflowpiv: ROS package for estimating surface flow velocities in rivers via PIV";
    pkgXml              =   readstruct("package.xml");
    pkgXml.name         =   packName;
    pkgXml.version      =   pkgVer;
    pkgXml.description  =   pkgDesc;
    pkgXml.license      =   "BSD";
    pkgXml.maintainer.emailAttribute    =   "cjl@usgs.gov";
    pkgXml.maintainer.Text              =   "Carl J. Legleiter";
    writestruct(pkgXml,"package.xml","StructNodeName","package")
    % Need to remove text child tag from maintainer field to be compatible with
    % catkin_make
    tmpIn   =   readlines("package.xml");
    old     =   "<Text>"+pkgXml.maintainer.Text+"</Text>";
    new     =   pkgXml.maintainer.Text;
    tmpOut  =   replace(tmpIn,old,new);
    writelines(tmpOut,"package.xml")
end


%% Add a new line to pkgDir/src/CMakeLists.txt to list the subdirectory for this node
cMakeDirListFile    =   pkgDir+filesep+"src"+filesep+"CMakeLists.txt";
subDirList          =   "add_subdirectory(" +funcName +")";
% Check if we've already added this directory
if exist(cMakeDirListFile,"file") == 0
    % This file doesn't exist yet, so make the file and add this directory
    writelines(subDirList,cMakeDirListFile)
else
    % The file already exists so read it in and check whether this directory is
    % already included
    tmp             =   readlines(cMakeDirListFile);
    if ~any(contains(tmp,subDirList))
        writelines(subDirList,cMakeDirListFile,"WriteMode","append")
    end
end


%% Also need to make a new CMakeLists.txt file for the node, based on Michael's example
% Note that this is a copy of the file included in his zip archive from 1/26:
    % "I copied the existing template and put new lines at the bottom of
    % src/streamflowpiv/src/pivCore/CMakeLists.txt related to opencv."
nodeCmake   =   [...
    "set(NODE_NAME """ + funcName + """)"; ...
    "set(LIB_NAME ${NODE_NAME})"; ...
    "set(DIR_NAME ${NODE_NAME})"; ...
    "set(EXPORT ${NODE_NAME})"; ...
    "file(GLOB SOURCES"; ...
    "      ""*.cpp"""; ...
    "      ""*.c"""; ...
    ")"; ...        
    "file(GLOB HEADERS"; ...
    "      ""*.hpp"""; ...
    "      ""*.h"""; ...
    ")"; ...
    "add_executable(${NODE_NAME}"; ...
    "       ${SOURCES}"; ...
    ")"; ...        
    "add_dependencies(${NODE_NAME}"; ...
    "  ${catkin_EXPORTED_TARGETS}"; ...
    ")"; ...
    "target_link_libraries(${NODE_NAME}"; ...
    "  ${catkin_LIBRARIES}"; ...
    "  ${Boost_LIBRARIES}"; ...
    "  ${CMAKE_DL_LIBS}"; ...
    ")"; ...
    "set(USE_SYSTEM_OPENCV True)"; ...
    "#must contain OpenCVConfig.cmake or opencv-config.cmake"; ...
    "set(OpenCV_DIR C:/OpenCV/x64/vc16/lib)"; ... 
    "if(USE_SYSTEM_OPENCV)"; ...
    "#use system-installed opencv"; ...
    "  message(""Searching for opencv in ${OpenCV_DIR}"")"; ...
    " find_package(OpenCV REQUIRED)"; ...
    "  include_directories(${OpenCV_INCLUDE_DIRS})"; ...
    "  link_directories(${OpenCV_LIB_DIR})"; ...
    "  target_link_libraries(${NODE_NAME} ${OpenCV_LIBS})"; ...
    "elseif(WIN32)"; ...
    "  message(""Using local opencv libraries from ${PROJECT_SOURCE_DIR}/lib/"")"; ...
    "  add_library(coder_custom_lib_1 UNKNOWN IMPORTED)"; ...
    "  set_property(TARGET coder_custom_lib_1 PROPERTY IMPORTED_LOCATION ""${PROJECT_SOURCE_DIR}/lib/opencv_core452.lib"")"; ...
    "  add_library(coder_custom_lib_2 UNKNOWN IMPORTED)"; ...
    "  set_property(TARGET coder_custom_lib_2 PROPERTY IMPORTED_LOCATION ""${PROJECT_SOURCE_DIR}/lib/opencv_features2d452.lib"")"; ...
    "  add_library(coder_custom_lib_3 UNKNOWN IMPORTED)"; ...
    "  set_property(TARGET coder_custom_lib_3 PROPERTY IMPORTED_LOCATION ""${PROJECT_SOURCE_DIR}/lib/opencv_flann452.lib"")"; ...
    "  add_library(coder_custom_lib_4 UNKNOWN IMPORTED)"; ...
    "  set_property(TARGET coder_custom_lib_4 PROPERTY IMPORTED_LOCATION ""${PROJECT_SOURCE_DIR}/lib/opencv_imgproc452.lib"")"; ...
    "  target_link_libraries(${NODE_NAME}"; ...
    "    coder_custom_lib_1"; ...
    "    coder_custom_lib_2"; ...
    "    coder_custom_lib_3"; ...
    "    coder_custom_lib_4"; ...
    "  )"; ...
    "else()"; ...
    "  message(WARNING ""Not linking against opencv"")"; ...
    "endif()"; ...
    ];
writelines(nodeCmake,pkgDir+filesep+"src"+filesep+funcName+filesep+"CMakeLists.txt")


%% If *.yaml parameter file is specified, copy to package and make *.launch file
if exist("paramFile","var") && ~isempty(paramFile)
    % First copy the parameter file into the package folder structure
    % copyfile(matFuncDir+filesep+paramFile,pkgDir+filesep+paramFile);
    copyfile(fullfile(matFuncDir,paramFile),fullfile(pkgDir,paramFile));
    % Now programmatically create the corresponding launch file. Note that we
    % might be able to start the node from this launch file, too, but haven't
    % been able to get that to work as of 5/15/2023
    % Here's a working example that should also be in the same directory we just
    % copied the yaml file to:
        % <launch>
        %     <!--  <node name="rosPivEntry" pkg="steamflowpiv" type="rosPivEntry"/> -->
        %     <rosparam command="load" file="C:\catkin_ws\src\streamflowpiv\PivParams.yaml"/>
        % </launch>
    launchStr   =   char('<launch>', ...
                     '<!--  <node name="rosPivEntry" pkg="steamflowpiv" type="rosPivEntry"/> -->', ... This line is a comment we tried for starting the node, too
                     ['<rosparam command="load" file="' char(pkgDir) filesep char(paramFile) '"/>'], ...
                      '</launch>');
    writelines(string(launchStr),pkgDir+filesep+packName+".launch")
end


%% Check whether to test the node
if testFlag == 1
    %% Switch here based on selected computing platform:
    switch platform
        %% Windows
        case 'win'        
            cd(wsDir)
            %% Open a terminal and build the node
            clc
            % bigCmd  =   'wt -w 0 nt -d C:\catkin_ws\ --title build --colorScheme "Tango Light" --tabColor #FFFFFF';
            % system(bigCmd);
            disp('Enter the following command in a new ROS terminal window:')
            disp('  cd C:\catkin_ws')
            disp('  catkin_make')
            input("Once you have built the node, press enter to continue ...");
            
            %% Open a new tab in the same terminal window and start a ROS core
            clc
            % bigCmd  =   'wt -w 0 nt -d C:\catkin_ws\ --title roscore --colorScheme "Campbell" --tabColor #0C0C0C';
            % system(bigCmd);       
            disp('Enter the following command in a new ROS terminal:')
            disp('  roscore')
            input("Once you have started the ROS core, leave the terminal window open and press enter to continue ...");

            %% Play the bag file from a new ROS terminal 
            clc
            % bigCmd  =   'wt -w -1 -d C:\catkin_ws\ --title ROS --colorScheme "Solarized Light" --tabColor #FDF6E3';
            % system(bigCmd);        
            disp('Enter the following command in a new ROS terminal:')
            disp(['  rosbag play "' char(bag2play) '"'])
            disp('Press the spacebar to pause playback and keep this terminal open')
            input("Once you have executed these commands in the terminal, press enter to continue ...");         

            %% Copy openCV dll's from system to folder with executable for node
            % This is a critical final step if we're using anything from OpenCV
            % for image stabilization ...
            % Note that these are hard-coded paths specific to Carl's machine
            openCVdllDir    =   "C:\lib\install\opencv\x64\vc16\bin";
            nodeExeDir      =   "C:\catkin_ws\devel\lib\"+packName;
            copyfile(openCVdllDir+filesep+"*.dll",nodeExeDir);

            %% If a parameter file was provided, open a new panel in the terminal to call launch file for loading the parameters
            if exist("paramFile","var") && ~isempty(paramFile)
                clc
                % bigCmd  =   'wt -w 0 sp -v -d C:\catkin_ws\ --title lauch --colorScheme "Light" --tabColor #1DF2E3';
                % system(bigCmd);        
                disp('Enter the following commands in a new ROS terminal:')
                disp('  cd C:\catkin_ws')
                disp('  devel\setup.bat')
                disp("  roslaunch " + lower(packName) + " " + lower(packName) + ".launch")
                input("Once you have executed these commands in the terminal, press enter to continue ...");
            end
                
            %% Open a new pane in the terminal to run the node
            clc
            % bigCmd  =   'wt -w 0 sp -v -d C:\catkin_ws\ --title run --colorScheme "Dark" --tabColor #3DF7E3';
            % system(bigCmd);        
            disp('Enter the following commands in a new ROS terminal:')
            disp('  cd C:\catkin_ws')
            disp('  devel\setup.bat')
            disp("  rosrun " + lower(packName) + " " + funcName)
            disp("Once you have executed these commands in the current terminal,")
            disp("go back to the terminal with rosbag play, press spacebar to resume playback,")
            input("then return to this terminal and press enter to continue ...");

            %% One more ROS terminal and echo the output from the node
            clc
            % bigCmd  =   'wt -w 0 sp -v -d C:\catkin_ws\ --title echo --colorScheme "Solarized Dark" --tabColor #330011';
            % system(bigCmd);
            topic   =   input("Enter name of output topic from the new node to echo here: ","s");
            disp('Open a new ROS terminal and enter the following commands:')
            disp('  cd C:\catkin_ws')
            disp('    devel\setup.bat')
            disp(['    rostopic echo /' topic])

        case 'vm'
            error("virtual machine case not fully implemented yet but see comments in addRosNode.m for some initial guidance ...")
            % %% Copy the codegen-generated files in C:\catkin_ws\src to VM shared folder
            % % For VM, we set up a shared folder on the local, host, Windows PC that
            % % we can see on the VM as well: \CJLmfiles\TRiVIA\RiOS. Within this is a
            % % subfolder called forVM, so let's copy the files we need to this folder
            % % so we can get to them on the VM
            % vmShare     =   [getenv('o') 'CJLmfiles\TRiVIA\RiOS\forVM\'];
            % copyfile('C:\catkin_ws\src',[vmShare 'src']); % destination
            % 
            % %% Open a VM ROS terminal and build the corresponding node using catkin      
            % disp('On the Virtual Machine (VM), open a terminal and enter the following commands:')
            % disp('  cp -r /mnt/hgfs/RiOS/forVM/src/ ~/catkin_ws/src/')
            % disp('  If the command above does not work, you can "manually" drag and drop files into the VM')
            % disp('  cd ~/catkin_ws')
            % disp('  catkin_make')
            % disp('  source devel/setup.bash')          
            % disp('Leave this terminal open and keep it visible on your VM desktop')        
            % 
            % %% Open a VM terminal and start roscore
            % disp('Open a new terminal tab (CTRL+SHIFT+T) on the VM and enter the following command:')
            % disp('  roscore')
            % disp('Leave this terminal tab open, but create another new tab')
            % 
            % %% Play the bag file from a new VM terminal 
            % % rosbag play ~/bags/bag6xs600stable.bag   
            % [~,bagName] =   fileparts(bag2play);
            % disp('Enter the following command in a new VM terminal tab:')
            % disp(['  rosbag play ~/bags/' char(bagName) '.bag'])
            % disp('Press the space bar to pause the playback of the bag file')
            % disp('Leave this terminal tab open, but create another new tab')
            % 
            % %% In the other VM terminal, run the node we just generated
            % disp('Enter the following commands in the first VM terminal tab:')
            % disp('      source devel/setup.bash')
            % disp(['     rosrun ' char(packName) ' ' char(funcName)])
            % 
            % %% To dispay the output from the node, we can use a python script
            % % Copy C:\Users\cjl\OneDrive - DOI\UASPIV\Python\previewImgMsg.py to the
            % % VM in ~/catkin_ws, then in a VM terminal in catkin_ws ...
            % disp('To visualize the output from this ROS node, you can either run a Python script:')
            % disp('  python3 ~/catkin_ws/previewImgMsg.py')
            % disp('... or use another ROS package like this (replacing /topicName as needed):')
            % disp('  rosrun image_view image_view image:=/topicName')
    
    otherwise
        error("The input platform must be 'win' or 'vm'")

    end % Switch over platforms
end % if for the testFlag

%% Return to original directory
cd(matFuncDir)