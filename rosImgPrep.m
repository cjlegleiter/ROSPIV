function rosImgPrep
% rosImgPrep.m: Prepare image pair for PIV: mask, stabilize, filter, and enhance
%#codegen
%
%% rosImgPrep.m:
%   Preliminary to prepare a pair of images for PIV, including masking the
%   channel, stabilization, spatial smoothing, and contrast enhancement.
%
%% SYNTAX:
%   rosImgPrep
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of timeout, window, and possibly others.
%   These parameters can be read programatically from a file named
%   PivParams.yaml using a ROS launch file (see addRosNode.m and pivEntry.m for
%   details). The raw images from the sensor (or bag file during development)
%   are subscribed to on the topics named /raw1 and /raw2.  Note that the
%   encoding for the raw RGB images is hardwired as 'grbg'.
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the
%   pre-processed images to topics named /preProc1 and /preProc2. 
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure.
% > Also see rosReadRaw.m, the node that produces the inputs for this node.
% > Note that the encoding for the raw RGB images is hardwired as 'grbg'.
%
%% FUNCTION SUMMARY:
%   rosImgPrep

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 06/15/2023 - Merged elements of rosStabPair and rosPreProc into one prep node
% 06/17/2023 - Some attempts to streamline memory usage, output timing info
% 06/21/2023 - Include demosaic and rgb2gray steps that had been in rosReadRaw
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosImgPrep.m


%% Set up subscribers to receive images
imgSub1    =   rossubscriber('/raw1','sensor_msgs/Image','DataFormat','struct');
imgSub2    =   rossubscriber('/raw2','sensor_msgs/Image','DataFormat','struct');


%% Set up publishers for pre-procesed images and create messages
preProcPub1 =   rospublisher('/preProc1','sensor_msgs/Image','DataFormat','struct');
preProcMsg1 =   rosmessage(preProcPub1);
preProcPub2 =   rospublisher('/preProc2','sensor_msgs/Image','DataFormat','struct');
preProcMsg2 =   rosmessage(preProcPub2);

 
%% Get timeout, ptThresh, and window parameters from ROS server
timeout     =   rosparam("get","/timeout");
ptThresh    =   rosparam("get","/ptThresh");
window      =   rosparam("get","/window");


%% Hardwire encoding, which must be a "compile-time constant" to clear codegen
% Which implies that we can't read it in as a ROS parameter
encoding    =   'grbg';


%% Infinite while loop to receive raw image pairs, pre-process, and publish
tally       =   uint16(1);
disp("Ready to receive images and begin pre-processing ...")
while 1
    tStart  =   tic;
    %% Receive image pairs from the two subscribers
    [raw1msg,status1]   =   receive(imgSub1,timeout);
    [raw2msg,status2]   =   receive(imgSub2,timeout);
    % Bail out if we didn't get two images from the two subscribers
    if ~status1 || ~status2
        disp("An image pair was not received from the subscribers, so exiting ...")
        break
    end
    % Read in the raw data, which is stored as a vector, not a 2D image
    raw1    =   reshape(raw1msg.Data,[raw1msg.Width raw1msg.Height])';   
    raw2    =   reshape(raw2msg.Data,[raw2msg.Width raw2msg.Height])'; 
    % raw1    =   rosReadImage(raw1msg);
    % raw2    =   rosReadImage(raw2msg);

    %% Account for encoding and convert from RGB to gray
    % Note that the encoding can't be read in as a ROS parameter, so hardwired
    % above. The raw data are stored in a bayer pattern that has to be
    % demosaicked to make it into an RGB image. 
    % Remove Bayer filter pattern from raw data and "demosaic" to convert to
    % an RGB image
    raw1    =   demosaic(raw1,encoding);
    % Now convert from RGB to grayscale, which avoids the artifacts of the
    % Bayer filter pattern
    raw1    =   rgb2gray(raw1);
    % Now same kind of thing for the second image
    raw2    =   demosaic(raw2,encoding);    
    raw2    =   rgb2gray(raw2);

    %% Call helper function to create channel mask for each image
    disp("      ... masking ... ")
    mask1   =   makeMask(raw1);
    mask2   =   makeMask(raw2);

    %% Call helper function to stabilize image an pair using only land area in each
    disp("      ... stabilizing ... ")   
    [preProc1,preProc2] =   stabPair(raw1,raw2,mask1,mask2,ptThresh);

    %% Call helper function to enhance contrast and apply spatial filter to water-only images
    disp("      ... enhancing ... ")  
    preProc1=   imgEnhance(preProc1,window);
    preProc2=   imgEnhance(preProc2,window);

    %% Publish the two pre-processed images
    preProcMsg1         =   rosWriteImage(preProcMsg1,preProc1,"Encoding","mono8");
    send(preProcPub1,preProcMsg1);
    preProcMsg2         =   rosWriteImage(preProcMsg2,preProc2,"Encoding","mono8");    
    send(preProcPub2,preProcMsg2);       
    disp("Pre-processed images ready for PIV published to topics: /preProc1 and /preProc2")

    %% Report progress, increment counter, and move on
    disp("Image pre-processing completed, number of image pairs processed so far = " + string(tally));
    runTime     =   toc(tStart);
    disp("  ")
    disp("NODE rosImgPrep ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally       =   tally + 1;
end % while loop


%% HELPER FUNCTION TO CREATE CHANNEL MASK
function mask = makeMask(img)
%% Create and apply mask
% Automated water-only mask routine based on imgThresh2roi function within
% the TRiVIA app, which is essentially an Otsu-based threshold
mask        =   imbinarize(img,'adaptive','ForegroundPolarity','dark');
% Use bwconncomp to extract just the single largest object from this binary mask
% In the documentation for bwconncomp, see the example to erase largest
% component from image, but in our case we want to keep the largest component
CC          =   bwconncomp(mask);
% Get the number of pixels in each component
% The simplest and fastest way to do this would be cellfun, but can't codegen it
% so we'll have to use a loop over the components
numPixels   =   zeros(size(CC.PixelIdxList),"uint32");
for ii = 1:length(CC.PixelIdxList)
    numPixels(ii)   =   uint32(numel(CC.PixelIdxList{ii}));
end
% Find the index of the component with the largest number of pixels
[~,iBiggest]=   max(numPixels);
% Initialize the mask as an array of zeros like img
mask        =   zeros(size(img),"like",img);
% Within the largest component, replace the zeros in the mask with ones
mask(CC.PixelIdxList{iBiggest})    =   1;
% Apply a closing operation to get rid of small holes and clean this up a bit.
% Note that by using Inf as the final argument, the process will repeat until
% the image no longer changes, rather than imposing a set number of iterations
mask        =   bwmorph(mask,'close',Inf);

% % More efficient update suggested by Brett Shoelson of the MathWorks
% % Use regionprops to get the area of each of the "objects" (aka connected
% % components) in the initial binary mask
% Stats       =   regionprops(mask,"Area");
% % Use bwareaopen to remove any objects with a number of pixels less than the
% % specified value, which in our case is the number of pixels (i.e., area) in the
% % largest object --> assumed to be the water in the channel
% mask        =   uint8(bwareaopen(mask,max([Stats.Area])));


%% HELPER FUNCTION TO STABILIZE A PAIR OF FRAMES RELATIVE TO ONE ANOTHER USING LAND AREA ONLY
function [stab1,stab2] = stabPair(raw1,raw2,mask1,mask2,ptThresh)
% Use value for feature detection point threshold set as a parameter, but
% increase if necessary to reduce the number of points retained 
% Apply the masks to the images to isolate the land area, but note that the mask
% has 1's in the water so we want the opposite of the mask to get the land
land1       =   raw1.*uint8(~mask1);
land2       =   raw2.*uint8(~mask2);

%% Estimate transform from second frame to first frame using land area only ...
H           =   EstStabilizationTform(land1,land2,ptThresh);
% Then warp the second frame and its mask using this transformation ...
stab2       =   imwarp(raw2,affine2d(H),'OutputView',imref2d(size(raw2)));
mask2warp   =   imwarp(mask2,affine2d(H),'OutputView',imref2d(size(mask2)));

%% Apply channel masks to stabilized frames so that only water areas are processed further
stab1       =   raw1.*uint8(mask1);
stab2       =   stab2.*uint8(mask2warp);


%% HELPER FUNCTION TO GET INTER-IMAGE TRANSFORM AND ALIGNED POINT FEATURES
function H = EstStabilizationTform(raw1,raw2,ptThresh)
% Get inter-image transform and aligned point features.
%  H = EstStabilizationTform(raw1,raw2) returns an affine transform
%  between raw1 and raw2 using the |estimateGeometricTransform| function
%  H = cvexEstStabilizationTform(raw1,raw2,ptThresh) also accepts
%  arguments for the threshold to use for the corner detector
       
% Generate prospective points
pointsA     =   detectFASTFeatures(raw1, 'MinContrast', ptThresh);
pointsB     =   detectFASTFeatures(raw2, 'MinContrast', ptThresh);

% Select point correspondences
% Extract features for the corners
[featuresA,pointsA] =   extractFeatures(raw1, pointsA);
[featuresB,pointsB] =   extractFeatures(raw2, pointsB);

% Match features computed from the first and second images
indexPairs  =   matchFeatures(featuresA, featuresB);
pointsA     =   pointsA.Location(indexPairs(:,1),:);
pointsB     =   pointsB.Location(indexPairs(:,2),:);

% Use MSAC algorithm to compute the affine transformation
tform       =   estimateGeometricTransform(pointsB, pointsA, 'affine');
H           =   tform.T;


%% HELPER FUNCTION TO ENHANCE IMAGE CONTRAST AND APPLY SPATIAL FILTER
function img = imgEnhance(img,window)
%% Pre-process image with CLAHE and Wiener spatial filter
img         =   adapthisteq(img);    
img         =   wiener2(img,[window window]);