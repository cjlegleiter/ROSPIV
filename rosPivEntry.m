function rosPivEntry
% rosPivEntry.m: Import PIV parameters and simulated pre-processed image pairs
%#codegen
%
%% rosPivEntry.m:
%   Initial setup node to import PIV parameters and simulated pre-processed
%   image pairs to be used during PIV analysis carried out by other nodes
%
%% SYNTAX:
%   rosPivEntry
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of timeout. This parameter can be read
%   programatically from a file named PivParams.yaml using a ROS launch file
%   (see addRosNode.m and pivEntry.m for details). The simulated images from the
%   bag file are subscribed to on the topic named /camera/image_raw
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the
%   simulated pre-processed images to topics named /preProc1 and /preProc2. The
%   time stamp for each image is read in and used to compute the time interval
%   between the two frames. These time intervals are aggregated into a running
%   list, which is published to a the topic /deltaTlist.
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure
%
%% FUNCTION SUMMARY:
%   rosPivEntry

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/15/2023
% 05/24/2023 - Added time stamp list aggregation and publishing
% 06/07/2023 - Reintroduced rosrate and waitfor to read in images at a desired
%              rate and avoid time stamps from images 1 and 2 being too close
% 06/18/2023 - Update time interval calculation and streamline memory usage
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosPivEntry.m


%% See pivEntry.m for example inputs 


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


%% Set up publishers for pre-processed images and create messages
imgPub1     =   rospublisher('/preProc1','sensor_msgs/Image','DataFormat','struct');
preProc1    =   rosmessage(imgPub1);
imgPub2     =   rospublisher('/preProc2','sensor_msgs/Image','DataFormat','struct');
preProc2    =   rosmessage(imgPub2);


%% Set up publisher for list of time intervals between images and create message
deltaTpub   =   rospublisher('/deltaTlist','std_msgs/Float32MultiArray','DataFormat','struct');
deltaTlist  =   rosmessage(deltaTpub);

 
%% Get timeout from ROS parameter server
timeout     =   rosparam("get","/timeout");


%% Infinite while loop to receive simulated images and publish image pairs and time intervals
% ... receive images and publish output to ROS messages
tally       =   uint16(1);
disp("Ready to receive raw images and begin the PIV workflow...")
% Pre-allocate an excessively large array to hold time intervals between images
maxPairs    =   uint16(5000);
deltaTout   =   zeros(maxPairs,1);
% Set the status flags for reading both images to true to start out with
status1     =   true;
status2     =   true;
while status1 && status2 && tally<maxPairs
    %% Receive raw images and their time stamps from the same subscriber, one after another
    % Use a single subscriber rather than two separate subscribers (i.e., one
    % subscriber for each image), which allowed us to avoid reading the same
    % image twice
    % First get the first image
    [raw1msg,status1]   =   receive(imgSub,timeout);
    % Use the same subscriber to get the next image
    [raw2msg,status2]   =   receive(imgSub,timeout); 
    % Bail out if we didn't get two images from the subscriber
    if ~status1 || ~status2
        disp("An image pair was not received from the subscriber, so exiting ...")
        break
    end
    % Read in the raw data, which is stored as a vector, not a 2D image
    img1                =   reshape(raw1msg.Data,[raw1msg.Width raw1msg.Height])';
    % Now same kind of thing for the second image
    img2                =   reshape(raw2msg.Data,[raw2msg.Width raw2msg.Height])';
    
    % Also get the time stamps associated with each image    
    t1                  =   double(raw1msg.Header.Stamp.Sec) + 1e-9*double(raw1msg.Header.Stamp.Nsec);       
    t2                  =   double(raw2msg.Header.Stamp.Sec) + 1e-9*double(raw2msg.Header.Stamp.Nsec);
    % Display time interval between frames   
    disp("   --> Time interval between the two image frames = " + string(t2-t1))
    
    %% Aggregate list of time intervals between frames
    deltaTout(tally)    =   t2 - t1;
    
    %% Publish the pre-processed image pair and the lists of time intervals between images
    % %% Publish the pre-processed image pair and the two lists of time stamps
    preProc1                =   rosWriteImage(preProc1,img1,"Encoding","mono8");
    preProc1.Header.Stamp   =   rostime(t1,"DataFormat","struct");
    send(imgPub1,preProc1);
    preProc2                =   rosWriteImage(preProc2,img2,"Encoding","mono8");
    preProc2.Header.Stamp   =   rostime(t2,"DataFormat","struct");
    send(imgPub2,preProc2);    
    deltaTlist.Data         =   single(deltaTout(logical(deltaTout)));
    send(deltaTpub,deltaTlist);
    disp("Pre-processed images published to topics: /preProc1 and /preProc2")

    %% Report progress, increment counter, and move on
    disp("Image preparation completed, number of image pairs processed so far = " + string(tally));
    tally       =   tally + 1;
end % while loop