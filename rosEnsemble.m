function rosEnsemble
% rosEnsemble.m: Enable ensemble PIV by adding up correlation matrices as they are computed
%#codegen
%
%% rosEnsemble.m:
%   Aggregates and adds up correlation matrices as they are produced by
%   rosMergeFFT to form an ensemble that can serve as input to the peak-finding
%   node rosPeak
%
%% SYNTAX:
%   rosEnsemble
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of the required variables (see
%   rosSetupIA.m and rosPivEntry.m). As correlation matrices are produced by one
%   of  the FFT nodes, they are published to the topic /cor, to which this
%   function subscribes
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes the
%   ensemble correlation matrix to a topic named /ensCor
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure
% > Takes as input the output from rosMergeFFT.m, so see that function as well
% > Note that for now we will assume that the time interval between frames is
%   consistent so that we can just add up the correlation matrices
%
%% FUNCTION SUMMARY:
%   rosEnsemble

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/24/2023
% 06/07/2023 - Minor cleanup
% 06/19/2023 - Revisit to optimize run time and memory usage
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosEnsemble.m


%% Set up subscriber to receive correlation matrices
corSub      =   rossubscriber('/cor','std_msgs/Float32MultiArray','DataFormat','struct');


%% Set up publisher for ensembled correlation matrix and create message
ensCorPub   =   rospublisher('/ensCor','std_msgs/Float32MultiArray','DataFormat','struct');
ensCor      =   rosmessage(ensCorPub);


%% Get additional inputs from ROS parameter server
timeout     =   rosparam("get","/timeout");
% Query ROS parameter server to get the array dimensions
height      =   rosparam("get","/intArea");
width       =   rosparam("get","/intArea");
% We can get the total number of IAs using a couple of the parameters on the ROS
% server
layers      =   rosparam("get","/numX")*rosparam("get","/numY");


%% Infinite while loop to receive per-frame pair correlation matrices, add them up, and publish ensemble correlation matrix
tally       =   1;
disp("Ready to receive correlation matrices ...")
while 1    
    tStart          =   tic;
    %% Receive per-frame pair correlation matrix
    [corMsg,status] =   receive(corSub,timeout);
    % Bail out if we didn't read in anything
    if ~status
        disp("No correlation matrix available to read from the subscriber, so exiting ...")
        break
    end
    
    %% Concatenate correlation matrices as they come in        
    ensCorOut       =   zeros([height width layers],"single");
    if tally == 1
        % Use the first correlation matrix to come in to initialize the stack
        % Read in correlation matrix, which is stored as a vector not a 3D array
        ensCorOut   =   reshape(corMsg.Data,[height width layers]);
    else
        % For all subsequent matrices that come in, just add them to what we
        % already have. A simple addition is sufficient because we actually
        % don't need to average at all since the absolute magnitude of the
        % correlation peak (i.e., the z value) is not important, just the
        % so-called "x,y" location where the peak occurs. 
        ensCorOut   =   ensCorOut + reshape(corMsg.Data,[height width layers]);
    end
    
    %% Publish the correlation matrix aggregated over the ensemble   
    ensCor.Data     =   reshape(ensCorOut,[numel(ensCorOut) 1]);    
    send(ensCorPub,ensCor);       
    disp("Aggregated ensemble correlation matrix published to topic: /ensCor")

    %% Report progress, increment counter, and move on
    disp("Ensemble correlation matrix published, number of matrices aggregated so far = " + string(tally));
    runTime         =   toc(tStart);
    disp("  ")
    disp("NODE rosEnsemble ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally           =   tally + 1;
end % while loop