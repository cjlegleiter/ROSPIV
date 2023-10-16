function rosReadRaw
% rosReadRaw.m: Read in raw images and time stamps and publish image pair with desired frame interval
%#codegen
%
%% rosReadRaw.m:
%   Initial node to read in raw images as they are recorded by the sensor, along
%   with their time stamps, and publish an image pair at the desired frame rate.
%
%% SYNTAX:
%   rosReadRaw
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of timeout, frameInterval, and possibly
%   others. These parameters can be read programatically from a file named
%   PivParams.yaml using a ROS launch file (see addRosNode.m and pivEntry.m for
%   details). The raw images from the sensor (or bag file during development)
%   are subscribed to on the topic named /camera/image_raw. 
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the raw
%   image pair to topics named /raw1 and /raw2. The time stamp for each image is
%   read in and used to compute the time interval between the two frames. These
%   time intervals are aggregated into a running list, which is published to a
%   the topic /deltaTlist.
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure.
%
%% FUNCTION SUMMARY:
%   rosReadRaw

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 06/11/2023 - Mesh together pieces of rosPivEntry.m and first part of pivCore.m
% 06/17/2023 - Some attempts to streamline memory usage, output timing info
% 06/18/2023 - Revised frame rate check to ensure consistency
% 06/21/2023 - Stripped down to only pass along two images at desired frame rate
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosReadRaw.m


%% Use *.launch file from addRosNode.m to read PivParams.yaml and set ROS parameters
% Modified addRosNode.m to take the name of the parameter file (i.e.,
% PivParams.yaml) as input and then programmatically write a *.launch file
% within the ROS package folder structure that can be run from the ROS terminal
% to automatically read in the parameter file and populate the ROS parameter
% server with the parameter names and values specified in the file. For example
% inputs, see demoScript.m, then call:
    % addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play);
% If testFlag is true, addRosNode.m will prompt you through the steps, but just
% for the record, the key command to enter at the ROS terminal command line to
% read in the parameter file is:
    % roslaunch streamflowpiv streamflowpiv.launch    

    
%% Set up a subscriber to receive raw images (and their associated time stamps)
% Use a single subscriber to avoid reading same image twice
imgSub      =   rossubscriber('/camera/image_raw','sensor_msgs/Image','DataFormat','struct');


%% Set up publishers for raw images
rawPub1     =   rospublisher('/raw1','sensor_msgs/Image','DataFormat','struct');
rawPub2     =   rospublisher('/raw2','sensor_msgs/Image','DataFormat','struct');


%% Set up publisher for list of time intervals between images and create message
deltaTpub   =   rospublisher('/deltaTlist','std_msgs/Float32MultiArray','DataFormat','struct');
deltaTlist  =   rosmessage(deltaTpub);

 
%% Get timeout and desired frame interval from ROS parameter server
timeout     =   rosparam("get","/timeout");
frameInt    =   rosparam("get","/frameInterval");


%% Infinite while loop to receive raw RGB images and time stamps and publish pairs
% Initialize counter to tally up how many image pairs have been read in
tally       =   uint16(1);
disp("Ready to receive raw images and begin the PIV workflow...")
% Pre-allocate an excessively large array to hold time intervals between images
maxPairs    =   uint32(5000);
deltaTout   =   zeros(maxPairs,1);
% Set the status flags for reading both images to true to start out with
status1     =   true;
status2     =   true;
% Also initialize variables for the second image just so they will be defined on
% all execution paths and thus allow us to clear codegen
raw2msg     =   rosmessage(imgSub);
t2          =   uint32(0); %#ok<NASGU>
% Start the while loop 
while status1 && status2 && tally<maxPairs
    tStart              =   tic;
    deltaT              =   0;
    %% Receive raw images and their time stamps from the same subscriber, one after another
    % First get the first image
    [raw1msg,status1]   =   receive(imgSub,timeout);
    % Time stamps are stored natively as uint32 for both Sec and Nsec, so
    % multiply the Nsec by 1E-9 to get it to seconds, then add the Sec
    t1                  =   double(raw1msg.Header.Stamp.Sec) + 1e-9*double(raw1msg.Header.Stamp.Nsec);
    while status1 && status2 && ... both image reading status flags are true
          (frameInt - frameInt*0.1 >= deltaT) && ... deltaT is greater than 90% of desired frame interval
          (deltaT <= frameInt + frameInt*0.1) ... deltaT is less than 110% of desired frame interval           
        % Use the same subscriber to get the next image. By using the same
        % subscriber both times, we avoid reading in the same image twice in a
        % row
        [raw2msg,status2]   =   receive(imgSub,timeout);
        t2                  =   double(raw2msg.Header.Stamp.Sec) + 1e-9*double(raw2msg.Header.Stamp.Nsec);
        deltaT              =   t2 - t1;
    end
    
    %% Check whether we actually have two images
    if status1 && status2   
        %% Display time interval between frames   
        disp("   --> Time interval between the two image frames = " + string(deltaT))
        
        %% Aggregate list of time intervals between frames and remove outliers
        deltaTout(tally)=   deltaT;
        deltaTclean     =   deltaTout(logical(deltaTout));
        deltaTclean     =   deltaTclean(~isoutlier(deltaTclean));
    
        %% Publish raw images and list of time intervals between images
        % Just republish the same messages we received from the subscriber,
        % resampled to the desired frame interval
        send(rawPub1,raw1msg);
        send(rawPub2,raw2msg);
        % Along with the list of time periods between frames
        deltaTlist.Data         =   single(deltaTclean);
        send(deltaTpub,deltaTlist);
        disp("Raw images published to topics: /raw1 and /raw2")
    
        %% Report progress, increment counter, and move on
        disp("Raw image import completed, number of image pairs processed so far = " + string(tally));
        runTime         =   toc(tStart);
        disp("  ")
        disp("NODE rosReadRaw ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
        disp("  ")
        tally       =   tally + 1;   
    else
        disp("Image pair not received from subscriber --> exiting ...")
        break
    end
end % while loop