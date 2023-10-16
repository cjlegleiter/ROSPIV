function rosPeak
% rosPeak.m: Find peak of correlation surface to estimate particle displacement
%#codegen
%
%% rosPeak.m:
%   Given a correlation surface, first find the initial, integer- (pixel-) level
%   peak to infer the displacement vector. A local function within this function
%   then refines this estimate at a sub-pixel level. Can be applied to the
%   correlation matrix for an individual frame pair or to an ensemble
%   correlation matrix aggregated over time.
%
%% SYNTAX:
%   rosPeak
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of the required variables (see
%   rosSetupIA.m and rosPivEntry.m). For the ensemble implementation, the
%   aggregated correlation matrix published by the rosEnsemble node is
%   subscribed to on the topic named /ensCor.
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes vector
%   coordinates and components to the topic /pivOut as a 4-layer array with a
%   separate layer for x, y, u, and v
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure
% > Takes as input the output from rosEnsemble.m, so see that function as well
%
%% FUNCTION SUMMARY:
%   rosPeak

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/17/2023
% 05/24/2023 - Modified to allow for aggregated correlation matrices-->ensemble
% 06/07/2023 - Minor cleanup
% 06/19/2023 - Revisit to optimize run time and memory usage
% 06/22/2023 - Modify to have a single publisher with a 4-layer array: x,y,u,v 
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosPeak.m


%% Set up subscriber to receive the stack of ensembled correlation matrices for all IAs
corSub      =   rossubscriber('/ensCor','std_msgs/Float32MultiArray','DataFormat','struct');


%% Set up subscriber to receive lists of time intervals between images
deltaTsub   =   rossubscriber('/deltaTlist','std_msgs/Float32MultiArray','DataFormat','struct');


%% Set up publisher for peakfinding output as an array with x,y,u,v layers and create message
pivPub      =   rospublisher('/pivOut','std_msgs/Float32MultiArray','DataFormat','struct');
pivOut      =   rosmessage(pivPub);


%% Get additional inputs from ROS parameter server
timeout     =   rosparam("get","/timeout");
pixSize     =   rosparam("get","/pixSize");
intArea     =   rosparam("get","/intArea");
step        =   rosparam("get","/step");
minY        =   rosparam("get","/minY");
minX        =   rosparam("get","/minX");
maxY        =   rosparam("get","/maxY");
maxX        =   rosparam("get","/maxX");
subPixOffset=   rosparam("get","/subPixOffset");
subpixMode  =   rosparam("get","/subpixMode");
% Get the array dimensions
height      =   rosparam("get","/intArea");
width       =   rosparam("get","/intArea");
% We can get the number of interrogation areas using a couple of the parameters
layers      =   rosparam("get","/numX")*rosparam("get","/numY");

%%
%% Infinite while loop to receive correlation matrices, find peaks, and publish vectors
tally   =   1;
disp("Ready to receive ensembled correlation matrices ...")
while 1
    tStart          =   tic;
    %% Receive stack of ensembled correlation matrices for all IAs
    [corMsg,status] =   receive(corSub,timeout);  
    % Bail out if we didn't read in anything
    if ~status
        disp("No correlation matrices read from the subscriber, so exiting ...")
        break
    end
    % Read in the stacked correlation matrices, which are stored as a vector,
    % not a 3D array, so we have to reshape the data into a 3D array

    %% Call a local function to find peaks and infer vectors
    disp("Finding correlation surface peaks ...")
    [xTmp,yTmp,uTmp,vTmp]   =   peakfinding(reshape(corMsg.Data,[height width layers]),...
                                            intArea,minX,step,maxX,minY,maxY,...
                                            subPixOffset,subpixMode);

    %% Use time gap between frames and pixel size to scale vectors
    % This is the only place in the entire PIV workflow where the time
    % information comes into play, just at the very end to convert a
    % displacement per frame to a displacement per second.
    % Aggregate all the inter-frame time differences as they come in and take
    % the average to get a mean deltat. We cleaned up the subscription to and
    % publication of time stamps in rosPivEntry, so we shouldn't have duplicates
    % or skipped frames by the time we get to this node.
    [deltaTmsg,st1] =   receive(deltaTsub,timeout);
    if st1
        deltaTlist  =   deltaTmsg.Data;
    else
        disp("*** WARNING: no new time intervals being published to deltaTlist, using most recent listing")
        deltaTlist  =   deltaTsub.LatestMessage.Data;
    end
    disp("Mean time interval between frames = " + string(mean(deltaTlist)) + " s")
       
    %% Publish initial PIV results as float32 array with 4 layers
    % Consider outputting these as 16-bit integers by using the 16SC1 encoding
    % and converting the velocities to millimeters to second before export
    % Think about whether we even need to output the x and y since they're just
    % grids of the same dimensions as the vector components
    
    % A lot to unpack in this next line of code, but we can unravel as follows:
        % 1)Stack up the x, y, u, and v 2D matrices into a 3D array with four
        %   layers
        % 2)Note that each layer has to be converted to single precision
        % 3)The original vectors are rescaled from pixels/frame to m/s by
        %   multiplying by the pixel size and divding by the mean time interval
        %   between frames
        % 4)To make a 1D data vector as expected by the ROS message format,
        %   reshape the 3D array; we know the number of elements in one layer,
        %   so just multiply that by four and then the second dimension is just
        %   a one to make it a vector
    pivOut.Data =   reshape(cat(3,single(xTmp),single(yTmp),...
                                  single(uTmp*pixSize/mean(deltaTlist)),...
                                  single(vTmp*pixSize/mean(deltaTlist))),...
                            [4*numel(xTmp) 1]);
    send(pivPub,pivOut)

    %% Report progress, increment counter, and move on
    disp("Peak-finding completed, number of image pairs processed so far = " + string(tally));
    runTime =   toc(tStart);
    disp("  ")
    disp("NODE rosPeak ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally   =   tally + 1;
end % while loop

%%
%% **** HELPER FUNCTION FOR FINDING PEAK OF CORRELATION SURFACE, FIRST AT INTEGER LEVEL
function [xPiv,yPiv,uPiv,vPiv] = peakfinding(corStack,intArea,minX,step,maxX,...
                                             minY,maxY,subPixOffset,subpixMode)

%% Rescale correlation values to 0-255 range
% This gets the minimum correlation value for each layer of the stack of IAs and
% then rearranges it to be an IA X IA X nIAx*nIAy array with that value
% replicated for all elements of a given layer
minres      =   permute(repmat(squeeze(min(min(corStack))),...
                        [1, size(corStack,1), size(corStack,2)]),[2 3 1]);
% This does the same kind of thing to get the range of correlation values for
% each layer of the stack of IAs and then rearranges it to be an IA X IA X
% nIAx*nIAy array with that range value replicated for all elements of a given
% layer
deltares    =   permute(repmat(squeeze(max(max(corStack)) - min(min(corStack))),...
                        [1,size(corStack,1),size(corStack,2)]),[2 3 1]);
% And so this essentially rescales the correlation values to be in the range
% from 0 to 255, probably just to save space and avoid errors in dealing with
% huge numbers
corStack    =   ((corStack-minres)./deltares)*255;
% To take a look at one example ...
    % figure
    % imshow(result_conv_ensemble(:,:,iTmp),[]); colorbar; axis on; 
    % set(gca,'xTick',0:16:128,'YTick',0:16:128,'GridColor','y'); grid on

%% Find all places where the correlation is at its maximum value 
% Get the row,column,layer indices of all the locations where the
% correlation is at its maximum value, which is 255 after our rescaling
[yPiv,xPiv,z]   =   ind2sub(size(corStack),find(corStack==255));
    % For our example, this yields 369x1 vectors for yPiv,xPiv, and z. So
    % for example, result_conv_ensemble(yPiv(33),xPiv(33),z(33)) --> 255

%% Get a single peak from all the resulting candidates 
% We need only one peak from each frame pair
% Sort the layer indices and get the sorted indices, too
[z1,zi]     =   sort(z);
% Take the first one in the sorted list and tack on the difference in the
% indices for all the remaining entries in the sorted list
dz1         =   [z1(1); diff(z1)];
% This gets all the entries where the peaks are not identical
i0          =   find(dz1~=0);
x1          =   xPiv(zi(i0));
y1          =   yPiv(zi(i0));
z1          =   z(zi(i0));
% So this whole cell is just to detect duplicate peaks and keep only one,
% but in our example this didn't happen so x1 is the same as xPiv and likewise
% for yPiv and z, with all having 369 entries

%% Set up grids of x and y coordinates
% These two lines of code essentially function kind of like meshgrid and
% lead to arrays that are nIAy X nIAx
xPiv        =   repmat((minX:step:maxX)+intArea/2,length(minY:step:maxY),1);
yPiv        =   repmat(((minY:step:maxY)+intArea/2)',1,length(minX:step:maxX));

%% Call another local function to find the peak at a sub-pixel level
% Merge these into a single function and use subpixMode input as a switch for
% selecting the 1D or 2D fit
vector      =   subPixPeak(corStack,intArea,x1,y1,z1,subPixOffset,subpixMode);

%% Package outputs
% Once we have the sub-pixel peaks, a bit of rearrangement
vector      =   permute(reshape(vector,[size(xPiv') 2]),[2 1 3]);    
uPiv        =   vector(:,:,1);
vPiv        =   vector(:,:,2);

%%
%% **** HELPER FUNCTION FOR SUB-PIXEL PEAK-FINDING
function vector = subPixPeak(corStack,intArea,x1,y1,z1,subPixOffset,subpixMode)
% Look for inferred peaks that are outside the domain and drop them from list;
% note that this didn't come up for our test case and I don't see how it ever
% could, but doesn't hurt to leave it in
xi      =   find(~((x1<=(size(corStack,2)-1)) & (y1<=(size(corStack,1)-1)) & ...
                   (x1>=2) & (y1>=2)));
x1(xi)  =   [];
y1(xi)  =   [];
z1(xi)  =   [];

% Get maximum possible x coordinate, which is the number of columns in the array
xmax    =   size(corStack,2);

% Initialize output; an array with 2 columns (one for u, one for v) and a row
% for each of the IA's
vector  =   NaN(size(corStack,3),2);

% Check whether we have any peaks at all
if numel(x1)~=0
    switch subpixMode
        case 1 
            % Use the three-point Gaussian peak-fitting, which is the default
            % This version of the subpixel peak finder uses a three-point
            % Gaussian fit twice, once in the row direction and then once more
            % in the column dimension
            % Get linear indices of the peak locations
            ip      =   sub2ind(size(corStack),y1,x1,z1); 
            % The following 8 lines are copyright (c) 1998, Uri Shavit, Roi
            % Gurka, Alex Liberzon, Technion ï¿½ Israel Institute of Technology
                % http://urapiv.wordpress.com But note that this has become
                % openPIV: http://www.openpiv.net/
            % First get the peak along the y direction (rows) by comparing the
            % log of the correlation values for the inferred integer peak to the
            % values immediately before and after
            f0      =   log(corStack(ip));
            f1      =   log(corStack(ip-1));
            f2      =   log(corStack(ip+1));
            % Then calculate the peak 
            peaky   =   y1 + (f1-f2)./(2*f1-4*f0+2*f2);
            % Same kind of thing in the x direction (columns)
            f0      =   log(corStack(ip));
            f1      =   log(corStack(ip-xmax));
            f2      =   log(corStack(ip+xmax));
            peakx   =   x1 + (f1-f2)./(2*f1-4*f0+2*f2);
        case 2 
            % The other option is "2D" subpixel peak-finding
            % Initialize some arrays
            c10   =   zeros(3,3, length(z1));
            c01   =   c10;
            c11   =   c10;
            c20   =   c10;
            c02   =   c10;
            ip    =   sub2ind(size(corStack), y1, x1, z1);
            % Double for loop over row and column indices to fit Gaussian "dome"
            for i = -1:1
                for j = -1:1
                    %following 15 lines based on
                    %H. Nobach ï¿½ M. Honkanen (2005) Two-dimensional Gaussian
                    %regression for sub-pixel displacement estimation in
                    %particle image velocimetry or particle position estimation
                    %in particle tracking velocimetry Experiments in Fluids
                    %(2005) 38: 511ï¿½515
                    c10(j+2,i+2,:)   =   i*log(corStack(ip+xmax*i+j));
                    c01(j+2,i+2,:)   =   j*log(corStack(ip+xmax*i+j));
                    c11(j+2,i+2,:)   =   i*j*log(corStack(ip+xmax*i+j));
                    c20(j+2,i+2,:)   =   (3*i^2-2)*log(corStack(ip+xmax*i+j));
                    c02(j+2,i+2,:)   =   (3*j^2-2)*log(corStack(ip+xmax*i+j));
                    %c00(j+2,i+2)=(5-3*i^2-3*j^2)*log(result_conv_norm(maxY+j, maxX+i));
                end
            end
            c10     =   (1/6)*sum(sum(c10));
            c01     =   (1/6)*sum(sum(c01));
            c11     =   (1/4)*sum(sum(c11));
            c20     =   (1/6)*sum(sum(c20));
            c02     =   (1/6)*sum(sum(c02));
            %c00=(1/9)*sum(sum(c00));
            % Use this information to get the peak of the dome
            deltax  =   squeeze((c11.*c01-2*c10.*c02)./(4*c20.*c02-c11.^2));
            deltay  =   squeeze((c11.*c10-2*c01.*c20)./(4*c20.*c02-c11.^2));
            peakx   =   x1+single(deltax);
            peaky   =   y1+single(deltay);
        otherwise
            peakx   =   single(NaN);
            peaky   =   single(NaN);
    end
    % The rest is common to both the 1D and 2D cases    
    % Account for IA size and sub-pixel offsets
    SubpixelX   =   peakx-(intArea/2)-subPixOffset;
    SubpixelY   =   peaky-(intArea/2)-subPixOffset;
    % Output results to the vector
    vector(z1,:)=   [SubpixelX, SubpixelY];
end