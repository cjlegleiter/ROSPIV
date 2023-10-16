%% Demo script for ROSPIV
% Dr. Carl J. Legleiter, cjl@usgs.gov                         
% Observing Systems Division, Hydrologic Remote Sensing Branch
% United States Geological Survey                             
% 09/12/2023
% ~\TRiVIA\ROSPIV\demoScript.m


%% 1. Make a ROS *.bag file from an image sequence stored in a structure array
% This example is for a simulated image sequence generated using the Simulating
% Hydraulics and Images for Velocimetry and Refinement (SHIVER) framework
% described in Legleiter and Kinzel (in review). The file resulting from this
% process is named `SHIVER90cms1Hz.bag` and is available in the ROSPIV folder
% on the ROSPIV branch of the [TRIVIA] repository.

% Load a MATLAB *.mat data file that includes a structure array containing the
% image sequence to be made into a *.bag file
load("../pivImgSim/SimStack90cms.mat")
% The original simulated image sequence had a frame rate of 4 Hz, but we can
% resample this down to 1 Hz. Keep the full 60-second duration.
skipFactor  =   4;
StackSub    =   Stack1(1:4:end);
% Crop each frame to exclude the top of the reach (bottom of the image in
% row,column coordinates) so we don't include the particle void that develops by
% the end of the simulation at the top of the reach. We can also crop to exclude
% columns that are always outside the channel.
lastRow     =   2300;
colRange    =   1001:2000;
for ii = 1:length(StackSub)
    StackSub(ii).preProcSub =   StackSub(ii).preProc(1:lastRow,colRange);
end
% With this subsetting, the image dimensions are 2300 rows X 1000 columns
% This simulated data is PIV-ready, so the field we need is called preProcSub
imgFieldName    =   "preProcSub";
frameInterval   =   1;
bagFile         =   "SHIVER90cms1Hz.bag";
stack2bag(StackSub,imgFieldName,frameInterval,bagFile);
% Note that in general, much of this cell of code will not be necessary if the
% input image sequence stored in the structure array already has the desired
% frame rate and image dimensions. In other words, lines 19-33 above would not
% be necessary and you can skip straight to a call to the function stack2bag.m.


%% 2. Create a parameter file to use for ROS-based PIV of the simulated image sequence
% The helper function pivEntry.m was developed for this purpose and its
% documentation provides further details on the parameters we're setting here.
% Set PIV parameters for the *.bag file of SHIVER output created above:
PivParams.pixSize       =   0.15;   % meters
PivParams.frameInterval =   1;      % second
% Use the interrogation area size focused on in the SHIVER manuscript
PivParams.intAreas      =   48;
PivParams.subpixMode    =   1;
% Use filtering parameters from SHIVER manuscript
PivParams.minvel        =   0.05;
PivParams.maxvel        =   2.5;
PivParams.stdThresh     =   4;
PivParams.medianFilt    =   1.5;
PivParams.infillFlag    =   1;
PivParams.smoothFlag    =   1;
% Also set parameters used by ROS nodes
PivParams.timeout       =   60;
% Now call pivEntry.m to create the PivParams.yaml file that can be read in ROS
% to populate the parameter server
paramStr                =   pivEntry(PivParams);
% Copy the resulting PivParams.yaml file to C:\catkin_ws\src\streamflowpiv and
% then read these parameters into the ROS server using the following, with the
% final line used to view and confirm the parameter values:
    % cd C:\catkin_ws
    % devel\setup.bat
    % roslaunch streamflowpiv streamflowpiv.launch
    % rosparam get /


%% 3. Building the network of ROS nodes for PIV from MATLAB source code
% The following is an example showing how to build the first of the eight ROS
% nodes you will ultimately need.
% Assume we are already in the folder with the MATLAB source code we want to
% make into a ROS node, so we can specify it as the directory with the source
matFuncDir  =   cd;
% Specify the name of the ROS package
packName    =   "streamflowpiv";
% Specify name of *.m file to be made into a ROS node as a char, not a string
funcName    =   'rosPivEntry';
% Point to the parameter file we created earlier
paramFile   =   "PivParams.yaml";
% Logical flag to build and test the node, keep true even if you're not ready to
% do a full test yet so that the addRosNode.m function will still guide you
% through the build process
testFlag    =   true;
% Specify which platform you are using, only "win" supported so far
platform    =   "win";
% Point to the bag file to be used for testing, even if you won't do a full run
bag2play    =   "SHIVER90cms1Hz.bag";
% Now call addRosNode to go through the codegen process
addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play);
% As long as testFlag is true, addRosNode.m will prompt you through the steps to
% codegen and catkin_make a ROS node from the MATLAB source code, but you can
% just press CTRL+C to stop after this process is complete and not go through a
% full run.

% Note that you will need to run addRosNode.m for each function in turn to build
% the whole network of ROS nodes. For each one, leave testFlag as true but only
% do the initial catkin_make to build the node and then CTRL+C at the MATLAB
% prompt to stop at that point rather than going through the whole test process
% for each node. Note that you will have to switch back to the matFuncDir at the
% command line before repeating the process for the next node:
cd(matFuncDir)

% List of MATLAB functions to make into rosNodes:
    % rosPivEntry.m
    % rosReadRaw.m
    % rosImgPrep.m
    % rosGetIA.m
    % rosMergeFFT.m
    % rosEnsemble.m
    % rosPeak.m
    % rosPostProc.m


%% 4. Use the ROS network to perform PIV for simulated image sequence
% After building all eight ROS nodes, you can do a full PIV run for the
% simulated image sequence you wrote to a ROS bag file in cell #1 above. Please
% refer to the User's Guide document for further guidance on setting up the ROS
% network and running the nodes.

% To view the output from a ROS-based PIV run in MATLAB, run these setup
% commands in MATLAB:
setenv("ROS_HOSTNAME","localhost");
setenv("ROS_MASTER_URI",'http://localhost:11311');
setenv('ROS_IP','127.0.0.1');
rosinit
% And then just to confirm communication with the ROS master
rostopic list 

% Once you have initiated all the ROS nodes, you can call the MATLAB function
% showVectors.m to plot the output from each iteration of the PIV workflow
pivTopic  =   '/pivFiltOut'; % for filtered output after running rosPostProc or 
% pivTopic  =   '/pivOut'; % for initial output
[xGrid,yGrid,uGrid,vGrid]   =   showVectors(pivTopic);
xlabel("Image columns (pixels)")
ylabel("Image rows (pixels)")
title("ROS PIV output: Iteration #8")
c       =   findobj(gcf,"type","colorbar");
c.Label.String  =   "Velocity magnitude (m/s)";


%% 5. ROS-based PIV for a recorded bag file from the Sacramento River
% For the bag file with real dta from the Sacramento River XS, we will need to
% update the PivParams structure array and call pivEntry.m again to create a new
% PivParams.yaml file that we can then copy to the catkin workspace as described
% above.
% The pixel size for this data set was calculated as 0.078 m/pixel
PivParams.pixSize       =   0.078; % meters
PivParams.frameInterval =   1; % second
% Calculate the the interrogation area size corresponding to an ideal resolution
% of 3.6 m to make it comparable to the SHIVER run
PivParams.idealresolution=  3.6;
% Using code from the TRiVIA end user software application, we can calculate the
% target IA size , with a  factor of two to account for the IA being twice the
% step size (i.e., desired output vector spacing)
targetIA                =   ceil(PivParams.idealresolution/PivParams.pixSize)*2; % 94
PivParams.intAreas      =   targetIA;
PivParams.subpixMode    =   1;
% Update velocity filtering parameters
PivParams.minvel        =   0.1;
PivParams.maxvel        =   5;
PivParams.stdThresh     =   4;
PivParams.medianFilt    =   1.5;
PivParams.infillFlag    =   1;
PivParams.smoothFlag    =   1;
% Also set parameters used by ROS nodes
PivParams.timeout       =   60;
% For a real image sequence that will need to be imported, masked, and
% stabilized, we need to specify a few additional parameters
PivParams.encoding      =   "grbg";
PivParams.ptThresh      =   0.2;
PivParams.window        =   5;
% Now call pivEntry.m to create the PivParams.yaml file that can be read in ROS
% to populate the parameter server
paramStr                =   pivEntry(PivParams);
% Copy the resulting PivParams.yaml file to C:\catkin_ws\src\streamflowpiv and
% then read these parameters into the ROS server using ...
    % cd C:\catkin_ws
    % devel\setup.bat
    % roslaunch streamflowpiv streamflowpiv.launch

% Perform the PIV run and view the output in MATLAB as described in cell #4