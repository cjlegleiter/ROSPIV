function rosMergeFFT
% rosMergeFFT.m: Perform FFTs to produce correlation matrix from a 4D, merged IA stack
%#codegen
%
%% rosMergeFFT.m:
%   Performs Fast Fourier Transform (FFT) operations for a pair of image subsets
%   extracted for all of the interrogation areas (IAs) in the original images
%   and combined into a single 4D, merged IA stack to produce correlation
%   matrices as a key component of the PIV algorithm
%
%% SYNTAX:
%   rosMergeFFT
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of the required variables (see
%   rosGetIA.m and rosPivEntry.m). The stacked, merged image subsets for all
%   IA's published by the rosGetIA node are subscribed to on the topic named
%   /mergeIA
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the
%   correlation matrix to a topic named /cor 
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure
% > Takes as input the output from rosGetIA.m, so see that function as well
%
%% FUNCTION SUMMARY:
%   rosMergeFFT

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/16/2023
% 05/24/2023
% 06/03/2023 - Modified to use a 4D, merged stack of IAs from both images
% 06/07/2023 - Minor cleanup
% 06/19/2023 - Revisit to optimize run time and memory usage
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosMergeFFT.m


%% Set up subscriber to receive merged 4D array of IA stacks from both images
iaMerge     =   rossubscriber('/mergeIA','std_msgs/UInt8MultiArray','DataFormat','struct');


%% Set up publisher for FFT output (i.e., correlation matrix) and create message
% Because this is a 3D array with lots of layers (i.e., more than 4), we can't
% use one of the image message types available in ROS and will have to use a
% different message type capable of handling 3D arrays.
corPub1     =   rospublisher('/cor','std_msgs/Float32MultiArray','DataFormat','struct');
cor         =   rosmessage(corPub1);


%% Get additional inputs from ROS parameter server
timeout     =   rosparam("get","/timeout");
% Get the array dimensions
height      =   rosparam("get","/intArea");
width       =   rosparam("get","/intArea");
% We can get the total number of IAs using a couple of the parameters on the
% ROS server
layers      =   rosparam("get","/numX")*rosparam("get","/numY");
% And we know that the number of images will always be 2


%% Infinite while loop to receive merged IA stacks, perform FFTs, and publish correlation matrices
tally   =   1;
disp("Ready to receive 4D arrays with merged IA stacks ...")
% % Some plotting code for development purposes
% hFig    =   figure("Position",[50 50 1800 600]); %#ok<NASGU>
% hAx     =   zeros(3,1);
% hAx(1)  =   subplot(1,3,1);
% hAx(2)  =   subplot(1,3,2);
% hAx(3)  =   subplot(1,3,3);
% i       =   13330;
while 1
    tStart          =   tic;
    %% Receive 4D arrays with merged IA stacks from both images
    [iaMsg,status]  =   receive(iaMerge,timeout);
    % Bail out if we didn't read in anything
    if ~status
        disp("No IA stack read from the subscriber, so exiting ...")
        break
    end
    % Read in the merged 4D array of IA stacks, which is stored as a vector, and
    % reorganize the data vector into a 4D array
    iaStack         =   reshape(iaMsg.Data,[height width layers 2]);
       
    % And then pull apart the two IA stacks, one for each image, so that we can
    % proceed as we had been previously
    img1            =   iaStack(:,:,:,1);
    img2            =   iaStack(:,:,:,2);
    
    % % Some plotting code for development purposes
    % axes(hAx(1)) %#ok<*LAXES>
    % imshow(img1(:,:,i),[],"InitialMagnification","Fit")
    % colorbar
    % title("Image 1, IA #" + string(i))
    % axes(hAx(2))
    % imshow(img2(:,:,i),[],"InitialMagnification","Fit")
    % colorbar
    % title("Image 2, IA #" + string(i))

    % First get the indices of any image subsets (IAs) that are all zeros
    corOut          =   zeros(size(img1),"single");
    iBlank          =   squeeze(~any(img1,[1 2]) | ~any(img2,[1 2]));
    % Use this to make subsets of the two IA stacks that leave out the blanks    
    img1noBlank     =   img1(:,:,~iBlank);
    img2noBlank     =   img2(:,:,~iBlank);
    % Now do the FFT just for the IAs that actually have data    
    corOut(:,:,~iBlank) =   doFFTs(img1noBlank,img2noBlank);
      
    % % Some plotting code for development purposes
    % axes(hAx(3))
    % imshow(corOut(:,:,i),[],"InitialMagnification","Fit")
    % colorbar
    % title("Image 1 - Image 2 correlation matrix, IA #" + string(i))

    %% Publish the stack of correlation matrices
    % To reorganize the 3D stack of correlation matrices into a vector, use the
    % reshape function. Confirmed that the reshape operator produces the same
    % output as the array organization specified in the ROS multiarray layout
    % documentation: Height is first dimension, then width, then layers
    % So to unravel the data vector later, this is what we'll need:
        % [height,width,layers]  =   size(corOut);
        % corFromVec             =   reshape(corVec,[height width layers]);
    % Now we need to assemble this into a ROS Float32MultiArray message    
    cor.Data        =   reshape(corOut,[numel(corOut) 1]); 
    % Finally, send the message to the publisher
    send(corPub1,cor);       
    disp("Stacked correlation matrices published to topic: /cor")

    %% Report progress, increment counter, and move on
    disp("Correlation matrices published, number of image pairs processed so far = " + string(tally));
    runTime         =   toc(tStart);
    disp("  ")
    disp("NODE rosMergeFFT ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally   =   tally + 1;
end % while loop

%%
%% HELPER FUNCTION FOR ACTUALLY DOING THE FFTS TO PRODUCE CORRELATION MATRICES
function corOut = doFFTs(img1,img2)
% Perform FFTs using MATLAB's built-in fftshift, ifft2, and fft2 functions. This
% is the real central core of the core piece of the whole workflowanalogous to
% the xcorrelate function used by GIV. A lot is wrapped up in this one line of
% code, but basically, working from the inside out we can unravel as follows:
% This is the bomb drop from the original PIVlab:
%     result_conv     =   fftshift(fftshift(real(ifft2(conj(fft2(image1_cut)).*...
%                                                fft2(image2_cut))), 1), 2);
% % And here's our step-by-step deconstruction:
% disp("Exectuting local function to perform FFT operations ...")
% % 1) Calculate FFT of both images
% fftIm1          =   fft2(img1);
% fftIm2          =   fft2(img2);
% 
% % 2) Take the complex conjugate of the FFT for the first image
% fftIm1conj      =   conj(fftIm1);
% 
% % 3) Element-by-element multiplication of the complex conjugate of the FFT
% %    for the first image by the FFT of the second image
% fftProd         =   fftIm1conj.*fftIm2;
% 
% % 4) Take the inverse FFT of this product
% invFftProd      =   ifft2(fftProd);
% 
% % 5) Keep the real component of this result
% realInvFftProd  =   real(invFftProd);
% 
% % 6) First call to built-in MATLAB function fftshift serves to "shift
% %    zero-frequency component to center of spectrum" along the row dimension,
% %    which is the ..., 1 near the end of the mega-line of code above. A second
% %    call to fftshift does the same thing in the column dimension, which is the
% %    ..., 2 at the very end of the mega-line of code above. Basically, this
% %    serves to re-center the output so the correlation peak at a lag of 0 is
% %    in the middle of the array
% shiftFftResult1 =   fftshift(realInvFftProd,1);
% shiftFftResult2 =   fftshift(shiftFftResult1,2);
% 
% % 7) The result of all this is a IA X IA X nIAy*nIAY 3D array 
% % with the correlation matrix for each IA stacked over the third dimension.
% corOut     =   shiftFftResult2;

% Collapse all of the above into one monster line of code and convert the input
% images to single precision so that the output will also be single precision
corOut      =   fftshift(fftshift(real(ifft2(conj(fft2(single(img1))).*...
                                                  fft2(single(img2)))),1),2);
% disp("FFT operations complete ...")