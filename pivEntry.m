function paramStr = pivEntry(PivParams)
% pivEntry.m: Begin modular, ROS-based approach to PIV analysis by loading parameters
%
%% pivEntry.m:
%   Starting point for a modular, ROS-based approach to PIV analysis. The
%   workflow begins by importing PIV parameters from a MATLAB structure array
%   and writing them to a file called PivParams.yaml that can be read into ROS
%   programmatically and used to populate the parameter server. Also includes
%   the timeout parameter to be used by the ROS image subscribers.
%
%% SYNTAX:
%   paramStr    =   pivEntry(PivParams);
%
%% INPUTS:
%   PIVparams:      Structure array with PIV input parameters in the fields:
%       .pixSize:   Pixel size of the input image sequence in meters
%       .frameInterval:  Original capture interval for the images, in seconds
%       .intAreas:  Size of the interrogation area in pixels
%       .subpixMode:scalar integer, either 1 or 2, specifying which mode to use
%                   for sub-pixel peak-finding on correlation matrices; 1 to use
%                   a 1D Gaussian fit applied separately to the row and column
%                   dimensions, or 2 to use a 2D Gaussian fit that considers
%                   both the row and column dimensions simultaneously
%       .minvel:    Minimum velocity threshold used to filter out suspiciously 
%                   low velocity estimates, in m/s. If an empty array or 
%                   negative number is passed as input, no filter is applied
%       .maxvel:    Maximum velocity threshold used to filter out suspiciously
%                   high velocity estimates, in m/s. If an empty array or
%                   negative number is passed as input, no filter is applied
%       .stdThresh: Threshold number of standard deviations, calculated at the
%                   reach scale, beyond which a velocity estimate will be 
%                   considered an outlier and filtered out (i.e., replaced with 
%                   NaN). If an empty array or negative number is passed as 
%                   input, no filter is applied
%       .medianFilt:Threshold value of the difference between a velocity
%                   estimate and a local median beyond which the estimate will 
%                   will be considered an outlier and filtered out (i.e., 
%                   replaced with NaN). If an empty array or negative number is
%                   passed as input, no filter is applied
%       .infillFlag:Logical flag to infill missing values in initial PIV output
%                   from each pass. Input should be true or 1 to perform 
%                   infilling, or if an empty array, 0, or negative number is 
%                   passed as input, no infilling is applied
%       .smoothFlag:Logical flag to smooth missing values in initial PIV output 
%                   from each pass. Input should be true or 1 to perform 
%                   smoothing, or if an empty array, 0, or negative number is 
%                   passed as input, no smoothing is applied
%       .timeout:   scalar specifying timeout delay to wait for images to appear
%                   on the ROS subscriber, in units of seconds (e.g., 5)
%       .encoding:  string specifying encoding of raw images, typically 'grbg'
%       .ptThresh:  scalar specifying the feature detection point threshold used
%                   for image stabilization. A reasonable default value is 0.2,
%                   but this can be increased if necessary to reduce the number
%                   of points retained.
%       .window:    scalar specifying size of square window used by Wiener
%                   spatial smoothing filter. A good default value is 5.
% 
%% OUTPUTS:
%   paramStr:       String array with the parameter names and values to be used
%                   to populate the ROS parameter server; this string is also
%                   written to the file PivParams.yaml in the current directory
%
%% NOTES:
%   > Call before addRosNode.m, which includes a section to read in the
%     PivParams.yaml file, copy it into the ROS package folder structure, and
%     create a launch file to load the parameters into ROS so they can be used
%     by subsequent ROS nodes. The key command to enter at the ROS terminal
%     command line to read in the parameter file is:
%           roslaunch streamflowpiv streamflowpiv.launch
%
%% FUNCTION SUMMARY:
%   paramStr    =   pivEntry(PivParams);

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov            
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey                
% 05/15/2023
% 06/07/2023 - Cleanup to include all parameters in PivParams structure,
%              including rhe number of slices to use in rosPyFFTmulti.py
% 09/06/2023 - Updated and streamlined for new ROSPIV branch
% ~\TRiVIA\ROSPIV\pivEntry.m


%% Example input for PivParams
% PivParams.pixSize       =   0.15;
% PivParams.frameInterval =   1; 
% PivParams.intAreas      =   28;
% PivParams.subpixMode    =   1;
% % Use default filtering parameters from TRiVIA
% PivParams.minvel        =   0.01;
% PivParams.maxvel        =   5;
% PivParams.stdThresh     =   4;
% PivParams.medianFilt    =   1.5;
% PivParams.infillFlag    =   1;
% PivParams.smoothFlag    =   1;
% % Also set parameters used by ROS nodes
% PivParams.timeout       =   30;
% PivParams.encoding      =   'grbg';
% PivParams.ptThresh      =   0.2;
% PivParams.window        =   5;


%% Parse interrogation area and set step size
% NOTE: Only single pass case implemented so far for other nodes
% % Assume that for each pass, including the first, the step size (i.e., output
% % vector spacing) will be half the interrogation area (IA) size for that pass.
intAreas    =   PivParams.intAreas;
intArea     =   intAreas(1);
step        =   intArea/2;


%% Example code to use PivParams to populate ROS parameter server
% % See the MATLAB documentation for rosparam
% % Simplest way to set a parameter is like this:
% % rosparam("set","intArea",intArea)
% % rosparam("set","step",step)
% % But the parameter tree is more general, so create one like so ...
% ptree       =   rosparam;
% % Then populate it with the fields in PivParams
% paramList   =   string(fieldnames(PivParams));
% for ii = 1:length(paramList)
%     if ~isempty(PivParams.(paramList(ii)))
%         set(ptree,paramList(ii),PivParams.(paramList(ii)));
%     end
% end
% set(ptree,"intArea",intArea)
% set(ptree,"step",step)
% % To avoid confusion, delete the intAreas parameter we already parsed above
% rosparam("delete","intAreas")
% % disp(ptree.AvailableParameters)
% % rosparam("get","/")
% % Also confirmed that this will work in a terminal after starting a roscore: 
%     % rosparam list
%     % rosparam get /


%% Write the parameter values to a *.yaml file that a ROS node can read
% Alternatively, we can make a .yaml file with parameters we can read in
% See this site: http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams#Using_rosparam
% And maybe this one too: https://foxglove.dev/blog/how-to-use-ros1-parameters

% Get list of parameters
paramList   =   string(fieldnames(PivParams));
% Set up an empty string to hold their names and values
paramStr    =   [];
% Loop over list of parameters
for ii = 1:length(paramList)
    % Don't include anything that was left empty and assume everything is a
    % double data type so we have to tack on the .0 so it will be recognized as
    % such
    if ~isempty(PivParams.(paramList(ii)))       
        tmpStr  =   paramList(ii) + ":  " + string(PivParams.(paramList(ii)));
        if ~contains(tmpStr,".") 
            tmpStr  =   tmpStr + ".0";
        end
        paramStr=   [paramStr; tmpStr]; %#ok<AGROW>
    end
end
tmpStr  =   "intArea:  " + string(intArea);
if ~contains(tmpStr,".")
    tmpStr  =   tmpStr + ".0";
end
paramStr=   [paramStr; tmpStr];
tmpStr  =   "step:  " + string(step);
if ~contains(tmpStr,".")
    tmpStr  =   tmpStr + ".0";
end
paramStr=   [paramStr; tmpStr];
% To avoid confusion, remove the intAreas parameter
paramStr(startsWith(paramStr,"intAreas"))   =   [];

% Hack to not tack on an extra 0 on the encoding line
if any(startsWith(paramStr,"encoding"))
    paramStr(startsWith(paramStr,"encoding"))   =   [];
    tmpStr  =   "encoding:  " + string(PivParams.encoding);
    paramStr=   [paramStr; tmpStr];
end

% Write out to a file
writelines(paramStr,"PivParams.yaml","LineEnding","\n","TrailingLineEndingRule","never");


%% Next step: use addRosNode.m to programmatically write a launch file to read parameters into ROS
% Modified addRosNode.m to take the name of the parameter file (i.e.,
% PivParams.yaml) as input and then programmatically write a *.launch file
% within the ROS package folder structure that can be run from the ROS terminal
% to automatically read in the parameter file and populate the ROS parameter
% server with the parameter names and values specified in the file. For example:
    % matFuncDir  =   fullfile(getenv("OneDrive"),"\CJLmfiles\TRiVIA\RiOS\MATLAB";
    % packName    =   "streamflowpiv";
    % funcName    =   'rosPivEntry'; % Note that this one is a char
    % paramFile   =   "PivParams.yaml";
    % testFlag    =   false;
    % platform    =   "win";
    % bag2play    =   fullfile(getenv("OneDrive"),"\CJLmfiles\TRiVIA\RiOS\MATLAB","60SHIVER.bag");
    % addRosNode(matFuncDir,packName,funcName,paramFile,testFlag,platform,bag2play);
% If testFlag is true, addRosNode.m will prompt you through the steps, but just
% for the record, the key command to enter at the ROS terminal command line to
% read in the parameter file is:
    % roslaunch streamflowpiv streamflowpiv.launch


%% Some old instructions on how to load the parameter file in a ROS terminal
% Just use this command:
    % rosparam load PivParams.yaml
% Then to display all the parameters and their values:    
    % rosparam get /
% Looks like this will work quite nicely, so could be a convenient way to load
% parameters without having to use a MATLAB structure array for PivParams


%% Some old example code to read parameters from yaml file and set them in ROS
% rosinit
% paramIn     =   readlines("PivParams.yaml");
% for ii = 1:length(paramIn)
%     paramName   =   extractBefore(paramIn(ii),":");
%     paramValue  =   str2double(extractAfter(paramIn(ii),":"));
%     if ~ismissing(paramName)
%         rosparam("set",paramName,paramValue);
%     end
% end