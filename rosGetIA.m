function rosGetIA
% rosGetIA.m: Get interrogation areas from full images for use in PIV algorithm
%#codegen
%
%% rosGetIA.m:
%   Sets up and extracts interrogation areas (IAs) from the full images for use
%   by the PIV algorithm based on a pair of pre-processed input images and
%   parameters for IA size and step size
%
%% SYNTAX:
%   rosGetIA
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of intArea and step. These parameters
%   can be read programatically from a file named PivParams.yaml using a ROS
%   launch file (see addRosNode.m and pivEntry.m for details). The pre-processed
%   images published by the rosPivEntry node are subscribed to on the topics
%   named /preProc1 and /preProc2
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the IA
%   stacks for both images, which are combined into a 4D array to a single topic
%   named /mergeIA. The following parameters are also added to the ROS parameter
%   server:
%       minY, minX, maxX, maxY, numY, numX, subPixOffset
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure
%
%% FUNCTION SUMMARY:
%   rosGetIA

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/15/2023 - Initial version as rosSetupIA.m
% 06/03/2023 - Revised to encompass both rosSetupIA and rosExtractIA in a single
%              node publishing a single topic that combines the IA stacks for
%              both images into a single 4D array
% 06/07/2023 - Minor cleanup
% 06/19/2023 - Revisit to optimize run time and memory usage
% 06/19/2023 - Convert to one combined local function to set up and extract IAs
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosGetIA.m


%% Set up subscribers to receive pre-processed images
imgSub1     =   rossubscriber('/preProc1','sensor_msgs/Image','DataFormat','struct');
imgSub2     =   rossubscriber('/preProc2','sensor_msgs/Image','DataFormat','struct');


%% Set up publisher for merged IA stack and create message
iaPub       =   rospublisher('/mergeIA','std_msgs/UInt8MultiArray','DataFormat','struct');
iaMsg       =   rosmessage(iaPub);

 
%% Get additional inputs from ROS parameter server
timeout     =   rosparam("get","/timeout");
intArea     =   rosparam("get","/intArea");
step        =   rosparam("get","/step");


%% Infinite while loop to receive image pairs and set up and publish merged IA stacks
tally   =   1;
disp("Ready to receive pre-processed images ...")
while 1
    tStart              =   tic;
    %% Receive pre-processed images
    [img1msg,status1]   =   receive(imgSub1,timeout);
    [img2msg,status2]   =   receive(imgSub2,timeout);
    % Bail out if we didn't read in anything
    if ~status1 && ~status2
        disp("No image read from either of the subscribers, so exiting ...")
        break
    end
    % Read in pre-processed images, which are stored as vectors, not 2D images
    img1                =   reshape(img1msg.Data,[img1msg.Width img1msg.Height])';       
    img2                =   reshape(img2msg.Data,[img2msg.Width img2msg.Height])';

    %% Call combined local function to set up interrogation areas and extract subsets
    [minY,minX,maxX,maxY,numY,numX,subPixOffset,iaMerge]=   makeIAstack(intArea,step,img1,img2);

    %% Publish outputs to ROS parameter server
    rosparam("set","minY",minY);
    rosparam("set","minX",minX);
    rosparam("set","maxY",maxY);
    rosparam("set","maxX",maxX);
    rosparam("set","numY",numY);
    rosparam("set","numX",numX);
    rosparam("set","subPixOffset",subPixOffset);
    
    %% Publish the merged 4D IA stack to a ROS topic
    % We can't use one of the image message types, so we need to create a ROS
    % message that can accommodate a 4D array. We set up an empty message of
    % type 'std_msgs/UInt8MultiArray' above, which has the following fields:
        % Data - this should be where the actual stack goes 
        % Layout - this is where we'll need to specify array dimensions etc. 
        % iaMsg.Layout.Dim has the fields
            % MessageType, Label, Size, Stride
    % Use the example on this site:
        % http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
    % To reorganize the 4D stack of image subsets (IAs) into a vector, use
    % the reshape function
    iaMerge             =   reshape(iaMerge,[numel(iaMerge) 1]);
    % Confirmed that the reshape operator produces the same output as the array
    % organization specified in the ROS multiarray layout documentation: Height
    % is first dimension, then width, then layers, then the two time periods as
    % the fourth dimension
    % So to unravel the data vector later, this is what we'll need:
        % [height,width,layers,nImage]=   size(iaMerge);
        % iaMergeFromVec              =   reshape(iaVec,[height width layers nImage]);
    % Rather than trying to specify the Layout part of the output message, which
    % doesn't seem to be read properly by the next node anyway, let's just use
    % the parameter server to specify the array dimensions, but we do need the
    % data, so ...
    iaMsg.Data          =   iaMerge;
    send(iaPub,iaMsg);
    disp("Merged 4D IA stack published to topic: /mergeIA")

    %% Report progress, increment counter, and move on
    disp("IA stacking completed, number of image pairs processed so far = " + string(tally));
    runTime         =   toc(tStart);
    disp("  ")
    disp("NODE rosGetIA ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally   =   tally + 1;
end % while loop


%% **** COMBINED HELPER FUNCTION TO SET UP INTERROGATION AREAS AND EXTRACT SUBSETS
function [minY,minX,maxX,maxY,numY,numX,subPixOffset,iaMerge] = makeIAstack(intArea,step,img1,img2)

disp("Setting up interrogation areas and padding images ...")
%% Get indices and dimensions of interrogation areas to extract from image
% These are the row,column pixel indices of the first IA to be extracted
% from the overall image
minY            =   1+(ceil(intArea/2));
minX            =   1+(ceil(intArea/2));
% Recall that step is always half the current IA
maxY           =   step*(floor(size(img1,1)/step))-(intArea-1)+(ceil(intArea/2));
maxX           =   step*(floor(size(img1,2)/step))-(intArea-1)+(ceil(intArea/2));

% Get the number of image subsets to be extracted from original image
numY    =   floor((maxY-minY)/step+1); % Number of IA's in the y direction
numX    =   floor((maxX-minX)/step+1); % Number of IA's in the x direction

% This has something to do with shifting the indices to avoid edge effects
LAy             =   minY;
LAx             =   minX;
LUy             =   size(img1,1)-maxY;
LUx             =   size(img1,2)-maxX;
shift4centery   =   round((LUy-LAy)/2);
shift4centerx   =   round((LUx-LAx)/2);
% Confusing comment from PIVlab source:
% shift4center will be negative if in the unshifted case the left border
% is bigger than the right border. the vectormatrix is hence not
% centered on the image. the matrix cannot be shifted more towards the
% left border because then image2_crop would have a negative index. The
% only way to center the matrix would be to remove a column of vectors
% on the right side. but then we weould have less data....
if shift4centery<0 
    shift4centery   =   0;
end
% Same kind of thing in the other direction
if shift4centerx<0 
    shift4centerx   =   0;
end

% Apply these shifts to update the IA pixel indices calculated above
minY           =   minY+shift4centery;
minX           =   minX+shift4centerx;
maxX           =   maxX+shift4centerx;
maxY           =   maxY+shift4centery;

%% Pad original images using built-in MATLAB function padarray
% Inputs are the original image, the amount to pad it by in both dimensions
% (the same amount in both dimensions as called here), and then the value to
% use for the padding, which is coded as the minimum in the image, so 0 in
% most cases including this one
img1           =   padarray(img1,[ceil(intArea/2) ceil(intArea/2)],min(min(img1)));
img2           =   padarray(img2,[ceil(intArea/2) ceil(intArea/2)],min(min(img1)));


%% Based on IA size (even or odd), specify offset for subpixel displacement
% This is just some fine-tuning for the final Gaussian peak-fitting after
% all the correlation matrices are calculated and then ensembled
if (rem(intArea,2) == 0)
    subPixOffset=1;
else
    subPixOffset=0.5;
end

%% Now extract subsets from overall image to populate interrogation areas
disp("Extracting subsets from images and stacking interrogation areas ...")
% Divide the images into subsets 
% Set up new indices for img1 and img2
% OK, so a lot to unpack here, but this yields an array of linear indices
% for carving out subsets of the original image
s0              =   (repmat((minY:step:maxY)'-1, 1,numX) + ...
                    repmat(((minX:step:maxX)-1)*size(img1, 1), numY,1))';
% This makes it into a 1 X 1 X nIAy*nIAY 3D array, with the indices in a vector
% in the third dimension
s0              =   permute(s0(:),[2 3 1]);
% This is an IA X IA array with some kind of image indices
s1              =   repmat((1:intArea)',1,intArea) + ...
                    repmat(((1:intArea)-1)*size(img1,1),intArea,1);
% This makes a big IA X IA X nIAy*nIAY 3D array of what must be linear indices
% that are what's used to actually carve out the subsets from the original image
ss1             =   repmat(s1,[1,1,size(s0,3)])+repmat(s0,[intArea,intArea,1]);
% Extract the image subsets (interrogation areas) This yields a IA X IA X
% nIAy*nIAY 3D array with the actual image subsets, one in each "layer" (third
% dimension). Stacking the individual image subsets for each IA in the third
% dimensionl like this is kind of a way of vectorizing the code and could lead
% to some time savings
img1            =   img1(ss1);
% For the first (and only) pass, we can use the same indices for the second
% image as well
img2            =   img2(ss1);

%% Finally, concatenate the two IA stacks into one 4D array
iaMerge         =   cat(4,img1,img2);