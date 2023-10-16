function rosPostProc
% rosPostProc.m: Post-process PIV output by filtering, infilling, and smoothing
%#codegen
%
%% rosPostProc.m:
%   Post-process the initial PIV output from rosPeak by applying multiple
%   filters: 1) velocity magnitude limits; 2) local median check; and 3)
%   standard deviation check. The filtered vectors can then be (optionally)
%   infilled to replace NaNs and (optionally) smoothed.
%
%% SYNTAX:
%   rosPostProc
%
%% INPUTS:
%   Function has no input arguments in the usual sense but assumes that the ROS
%   parameter server can provide values of the required variables (see
%   rosPivEntry.m). The initial vector components published by the rosPeak node
%   are subscribed to on the topic named /pivOut.
%
%% OUTPUTS:
%   Function has no output arguments in the usual sense but publishes filtered
%   vector coordinates and components to topic named /pivFilt
%
%% NOTES:
% > See also pivEntry.m, which sets up the parameter file, and addRosNode.m,
%   which ushers you through the codegen process and handles file management
%   within the ROS package directory structure.
% > Takes as input the output from rosPeak.m, so see that function as well.
%
%% FUNCTION SUMMARY:
%   rosPostProc

%% CREDITS:
% Dr. Carl J. Legleiter, cjl@usgs.gov
% Geomorphology and Sediment Transport Laboratory
% United States Geological Survey
% 05/25/2023
% 06/07/2023 - minor cleanup
% 06/19/2023 - Revisit to optimize run time and memory usage
% 06/23/2023 - Update to subscribe to and publish PIV results as 3D array with x,y,u,v layers
% 9/6/2023   - Check and cleanup for new ROSPIV branch
% ~\TRiVIA\ROSPIV\rosPostProc.m


%% Set up subscriber to receive the initial PIV output
pivSub      =   rossubscriber('/pivOut','std_msgs/Float32MultiArray','DataFormat','struct');


%% Set up publisher for filtered PIV output and create message
pivPub      =   rospublisher('/pivFilt','std_msgs/Float32MultiArray','DataFormat','struct');
pivFilt     =   rosmessage(pivPub);


%% Get additional inputs from ROS parameter server
timeout     =   rosparam("get","/timeout");
% Get the array dimensions
numX        =   rosparam("get","/numX"); % This is the width
numY        =   rosparam("get","/numY"); % This is the height
% Note that a negative number for any of these will skip the filtering step
minvel      =   rosparam("get","/minvel");
maxvel      =   rosparam("get","/maxvel");
stdThresh   =   rosparam("get","/stdThresh");
medianFilt  =   rosparam("get","/medianFilt");
infillFlag  =   rosparam("get","/infillFlag");
smoothFlag  =   rosparam("get","/smoothFlag");


%% Infinite while loop to receive initial vectors, filter, infill, and smooth
tally   =   1;
disp("Ready to receive initial velocity vectors ...")
while 1
    tStart          =   tic;
    %% Receive initial PIV output
    [pivMsg,status] =   receive(pivSub,timeout);
    % Bail out if we didn't read in anything
    if ~status
        disp("No initial PIV output read from the subscriber, so exiting ...")
        break
    end
    % Read in the PIV output, which is stored in a 3D array with 4 layers: x,y,u,v
    % Note that the vector components are already scaled by the time they get
    % here --- that is, pixel size and frame interval have been accounted for
    pivOut          =   reshape(pivMsg.Data,[numY numX 4]);
    xGrid           =   squeeze(pivOut(:,:,1));
    yGrid           =   squeeze(pivOut(:,:,2));
    uGrid           =   squeeze(pivOut(:,:,3));
    vGrid           =   squeeze(pivOut(:,:,4));

    %% 9/11/2023: NEW CODE TO RESET TO NAN A SPURIOUS DEFAULT VALUE ASSIGNED OUTSIDE THE CHANNEL
    % Get the mode of each vector component and use it to identify bad vectors
    % that were assigned a spurious but consistent default value outside the
    % channel and then reset those vectors to NaN's
    % First check whether more than 25% of the vectors all have the same modal
    % value before we jump into the correction
    maxModeFrac     =   0.25;
    % Divide the number of vectors that have the modal value by the total number
    % of vectors
    modeFrac        =   sum(sum(uGrid == mode(uGrid(:)))) / numel(uGrid);
    if modeFrac > maxModeFrac
        iBad            =   uGrid == mode(uGrid(:)) & vGrid == mode(vGrid(:));
        uGrid(iBad) =   NaN;
        vGrid(iBad) =   NaN;
    end

    %% 9/11/2023: NEW CODE TO FIND AREAS THAT ARE MOSTLY NANS AND PROBABLY OUTSIDE THE CHANNEL
    % Both uGrid and vGrid have NaN's in the same places so it doesn't matter
    % which we choose to use as the input to this algorithm and we only need to
    % run through this process once, not separately for uGrid and then again
    % later for vGrid
    in              =   uGrid;
    % Code from the GIV function nanfillsm, which has default input values of
    % nantolerance = 2 and smoothsize = 2, which we will assume here
    nantolerance    =   2;
    smoothsize      =   2;
    % Find the right NaNs
    % Make NaN values a very large negative number   
    nanvalue        =   -1e10;
    % Get number of elements based on smoothsise
    numelts         =   4;
    % Find the threshold value at which the average has too many nans
    thresholdvalue  =   (nanvalue*nantolerance + (numelts-nantolerance)*0)/numelts;
    in_working      =   in;
    in_working(isnan(in)) = nanvalue;
    % Make a mask of the proper size using code from the GIV function make_mask
        % [mask] = make_mask(smoothsize);
    mask_radius     =   floor(smoothsize/2);
    % Make a diamond shape
    mask            =   strel('diamond', mask_radius); 
    % Convert mask array from logical to double
    mask            =   double(mask.Neighborhood); 
    % Exclude the central element
    mask(mask_radius+1,mask_radius+1)   =   0; 
    % Identify where too many NaNs are present. This line of code must
    % essentially count the number of NaNs surrounding the current location
    in_working      =   conv2(in_working,mask,'same')/numelts;
    % Reset any location with too many NaNs to -999
    in_working(in_working<=thresholdvalue)  =   -999;
    % Now make anything that is not too many NaNs to a 0
    in_working(in_working~=-999)    =   0;
    % And annything that is too many NaNs to a 1
    in_working(in_working==-999)    =   1;
    % Copy the input array to a new variable
    in_nan          =   in;
    % Make anything that is not a nan in the original input a 10
    in_nan(~isnan(in))  =   10;
    % Now assign a 1 to any element of this array that had a Nan in the original
    % input array
    in_nan(isnan(in))   =   1;
        % At this point in_nan has a a 10 where not NaN and 1 where it is NaN
    % Get an array of bad nans by multiplying this by the working copy above
    in_nansbad          =   in_working.*in_nan;
    % Now this is a matrix with a 1 where NaNs are present AND are too close to
    % too many other NaNs
    in_nansbad(in_nansbad~=1)   =   0;
    % figure; imagesc(in_nansbad); axis equal; colorbar
    % Looks like this will work, providing us with an array that has 1's where
    % we will want to reset the output from the rest of this function to NaN to
    % act as a rough channel mask
    
    %% Call a local function to filter, infill, and smooth
    [uFilt,vFilt]   =   postproc(uGrid,vGrid,minvel,maxvel,stdThresh,medianFilt,...
                                 infillFlag,smoothFlag);

    %% 9/11/2023: NEW CODE TO APPLY THE INFERRED CHANNEL MASK FROM ABOVE
    uFilt(logical(in_nansbad))  =   NaN;
    vFilt(logical(in_nansbad))  =   NaN;
    % save tmp.mat uFilt vFilt
   
    %% Publish filtered PIV output as a float32 array with 4 layers
    % Consider outputting these as 16-bit integers by using the 16SC1 encoding
    % and converting the velocities to millimeters to second before export
    % Think about whether we even need to output the x and y since they're just
    % grids of the same dimensions as the vector components
    
    % A lot to unpack in this next line of code, but we can unravel as follows:
        % 1)Stack up the x, y, u, and v 2D matrices into a 3D array with four
        %   layers            
        % 2)To make a 1D data vector as expected by the ROS message format,
        %   reshape the 3D array; we know the number of elements in one layer,
        %   so just multiply that by four and then the second dimension is just
        %   a one to make it a vector
    pivFilt.Data    =   reshape(cat(3,xGrid,yGrid,uFilt,vFilt),[4*numel(xGrid) 1]);
    send(pivPub,pivFilt)

    %% Report progress, increment counter, and move on
    disp("Vector post-processing completed, number of image pairs processed so far = " + string(tally));
    runTime =   toc(tStart);
    disp("  ")
    disp("NODE rosPostProc ITERATION " + string(tally) + " RUN TIME = " + string(runTime));
    disp("  ")
    tally           =   tally + 1;
end % while loop

%%
%% **** HELPER FUNCTION FOR VECTOR POST-PROCESSING: FILTERING, INFILLING, SMOOTHING
function [uFilt,vFilt] = postproc(uGrid,vGrid,minvel,maxvel,stdThresh,medianFilt,...
                                  infillFlag,smoothFlag)
disp("**** Filtering initial output from PIV algorithm ****")

%% Velocity mangnitude limits
magScale    =   hypot(uGrid,vGrid);
if minvel > 0
    uGrid(magScale<minvel)  =   NaN;
    vGrid(magScale<minvel)  =   NaN;
    disp("Reset " + string(length(find(magScale<minvel))) + ...
         " vectors below minimum velocity threshold to NaN")
end
if maxvel > 0
    uGrid(magScale>maxvel)  =   NaN;
    vGrid(magScale>maxvel)  =   NaN;
    disp("Reset " + string(length(find(magScale>maxvel))) + ...
         " vectors above maximum velocity threshold to NaN")
end

%% Local median check
if medianFilt > 0
    % First for u component 
	neigh_filt                  =   medfilt2(uGrid,[3,3],'symmetric');
	neigh_filt                  =   abs(neigh_filt - uGrid);
	uGrid(neigh_filt>medianFilt)=   NaN;
    disp("Reset " + string(length(find(neigh_filt>medianFilt))) + ...
        " u components that failed local median check to NaN")
    
    % Same kind of thing for v component
	neigh_filt                  =   medfilt2(vGrid,[3,3],'symmetric');
	neigh_filt                  =   abs(neigh_filt-vGrid);
	vGrid(neigh_filt>medianFilt)=   NaN;
    disp("Reset " + string(length(find(neigh_filt>medianFilt))) + ...
        " v components that failed local median check to NaN")
end

%% Standard deviation check
if stdThresh > 0
    meanu               =   mean(uGrid(:),'omitnan');
    meanv               =   mean(vGrid(:),'omitnan');
    stdu                =   std(uGrid(:),0,'omitnan');
    stdv                =   std(vGrid(:),0,'omitnan');
    minvalu             =   meanu-stdThresh*stdu;
    maxvalu             =   meanu+stdThresh*stdu;
    minvalv             =   meanv-stdThresh*stdv;
    maxvalv             =   meanv+stdThresh*stdv;
    uGrid(uGrid<minvalu)=   NaN;
    disp("Reset " + string(length(find(uGrid<minvalu))) + ...
        " u components that failed standard deviation check (too low) to NaN")
    uGrid(uGrid>maxvalu)=   NaN;
    disp("Reset " + string(length(find(uGrid>maxvalu))) + ...
        " u components that failed standard deviation check (too high) to NaN")
    vGrid(vGrid<minvalv)=   NaN;
    disp("Reset " + string(length(find(vGrid<minvalu))) + ...
        " v components that failed standard deviation check (too low) to NaN")
    vGrid(vGrid>maxvalv)=   NaN;
    disp("Reset " + string(length(find(vGrid>maxvalu))) + ...
        " v components that failed standard deviation check (too high) to NaN")
end

%% Force any vector that has a nan for one component to have a nan for the other component as well
uGrid(isnan(vGrid))     =   NaN;
vGrid(isnan(uGrid))     =   NaN;

%% Replace nans using PIVlab function inpaint_nans.m
if infillFlag > 0
    disp("**** Infilling gaps in initial output from ensemble PIV algorithm ****")
    % Disable warning about a rank deficient matrix, which will probably only
    % come up if we have all (or mostly) NaN's for a particular IA, which could
    % happen a lot
    warning('off','MATLAB:rankDeficientMatrix')
    uPivInfill  =   inpaint_nans(uGrid,4); 
    vPivInfill  =   inpaint_nans(vGrid,4); 
else
    uPivInfill  =   uGrid;
    vPivInfill  =   vGrid;
end

%% Smooth final PIV output using PIVlab function smoothn.m
% The larger the input smoothing parameter s in the call to smoothn.m is, the
% smoother the output will be
if smoothFlag > 0
    disp("**** Smoothing initial output from ensemble PIV algorithm ****")    
    if infillFlag
        uPivSmooth  =   smoothn(double(uPivInfill),0.05);
        vPivSmooth  =   smoothn(double(vPivInfill),0.05);
    else
        uPivSmooth  =   smoothn(double(uGrid),0.05);
        vPivSmooth  =   smoothn(double(vGrid),0.05);
    end
else
    uPivSmooth      =   double(uGrid);
    vPivSmooth      =   double(vGrid);
end

%% Update output variables depending on what level of post-processing we've done
if infillFlag>0 && smoothFlag>0
    disp("Exporting infilled and smoothed output")
    uPivOut =   uPivSmooth;
    vPivOut =   vPivSmooth;
elseif infillFlag>0 && smoothFlag<=0
    disp("Exporting infilled but not smoothed output")
    uPivOut =   double(uPivInfill);
    vPivOut =   double(vPivInfill);
else
    disp("No infilling or smoothing applied, so exporting output without these procedures applied")
    uPivOut =   double(uGrid);
    vPivOut =   double(vGrid);
end

%% Output resulting filtered vectors
uFilt       =   uPivOut;
vFilt       =   vPivOut;
end % postproc helper function

%% Add smoothn (and its dependencies dctn and idctn) and inpaint_nans as subfunctions
function [z,s,exitflag] = smoothn(y,s)
% function [z,s,exitflag] = smoothn(varargin)
% %%%% 5/10/2023: SIMPLIFIED FOR CODEGEN BY CARL J. LEGLEITER

%SMOOTHN Robust spline smoothing for 1-D to N-D data.
%   SMOOTHN provides a fast, automatized and robust discretized smoothing
%   spline for data of any dimension.
%
%   Z = SMOOTHN(Y,S) smoothes the array Y using the smoothing parameter S.
%   S must be a real positive scalar. The larger S is, the smoother the
%   output will be. If the smoothing parameter S is omitted (see previous
%   option) or empty (i.e. S = []), it is automatically determined using
%   the generalized cross-validation (GCV) method.
%
%
%   [Z,S] = SMOOTHN(...) also returns the calculated value for S so that
%   you can fine-tune the smoothing subsequently if needed.
%%
%   [Z,S,EXITFLAG] = SMOOTHN(...) returns a boolean value EXITFLAG that
%   describes the exit condition of SMOOTHN:
%       1       SMOOTHN converged.
%       0       Maximum number of iterations was reached.
%
%   Class Support
%   -------------
%   Input array can be numeric or logical. The returned array is of class
%   double.
%
%   Notes
%   -----
%   The N-D (inverse) discrete cosine transform functions <a
%   href="matlab:web('http://www.biomecardio.com/matlab/dctn.html')"
%   >DCTN</a> and <a
%   href="matlab:web('http://www.biomecardio.com/matlab/idctn.html')"
%   >IDCTN</a> are required.
%
%   To be made
%   ----------
%   Estimate the confidence bands (see Wahba 1983, Nychka 1988).
%
%   Reference
%   --------- 
%   Garcia D, Robust smoothing of gridded data in one and higher dimensions
%   with missing values. Computational Statistics & Data Analysis, 2010. 
%   <a
%   href="matlab:web('http://www.biomecardio.com/pageshtm/publi/csda10.pdf')">PDF download</a>
%
%   See also DCTN and IDCTN.
%
%   -- Damien Garcia -- 2009/03, revised 2010/06
%   Visit my <a
%   href="matlab:web('http://www.biomecardio.com/matlab/smoothn.html')">website</a> for more details about SMOOTHN 


%% Test & prepare the variables
% y = array to be smoothed
y = double(y);
sizy = size(y);
noe = prod(sizy); % number of elements
if noe<2, z = y; return, end
% Smoothness parameter and weights
W = ones(sizy);
if ~isempty(s) && (~isscalar(s) || s<0)
    error('MATLAB:smoothn:IncorrectSmoothingParameter',...
        'The smoothing parameter must be a scalar >=0')
end
% "Maximal number of iterations" criterion
MaxIter = 100; % default value for MaxIter
% "Tolerance on smoothed output" criterion
TolZ = 1e-3; % default value for TolZ
% "Initial Guess" criterion
% isinitial = false; % default value for TolZ
% Weights. Zero weights are assigned to not finite values (Inf or NaN),
% (Inf/NaN values = missing data).
IsFinite = isfinite(y);
nof = nnz(IsFinite); % number of finite elements
W = W.*IsFinite;
if any(W<0)
    error('MATLAB:smoothn:NegativeWeights',...
        'Weights must all be >=0')
else 
    W = W/max(W(:));
end
%---
% Weighted or missing data?
isweighted = any(W(:)<1);
isauto     = false;
isrobust   = false;

%% Creation of the Lambda tensor
%---
% Lambda contains the eingenvalues of the difference matrix used in this
% penalized least squares process.
d = ndims(y);
Lambda = zeros(sizy);
for i = 1:d
    siz0 = ones(1,d);
    siz0(i) = sizy(i);
    Lambda = bsxfun(@plus,Lambda,...
        cos(pi*(reshape(1:sizy(i),siz0)-1)/sizy(i)));
end
Lambda = -2*(d-Lambda);
if ~isauto, Gamma = 1./(1+s*Lambda.^2); end

%% Upper and lower bound for the smoothness parameter
% The average leverage (h) is by definition in [0 1]. Weak smoothing occurs
% if h is close to 1, while over-smoothing appears when h is near 0. Upper
% and lower bounds for h are given to avoid under- or over-smoothing. See
% equation relating h to the smoothness parameter (Equation #12 in the
% referenced CSDA paper).
N = sum(sizy~=1); % tensor rank of the y-array
hMin = 1e-6; hMax = 0.99;
sMinBnd = (((1+sqrt(1+8*hMax.^(2/N)))/4./hMax.^(2/N)).^2-1)/16;
sMaxBnd = (((1+sqrt(1+8*hMin.^(2/N)))/4./hMin.^(2/N)).^2-1)/16;

%% Initialize before iterating
%---
Wtot = W;
%--- Initial conditions for z
if isweighted
    %--- With weighted/missing data
    % An initial guess is provided to ensure faster convergence. For that
    % purpose, a nearest neighbor interpolation followed by a coarse
    % smoothing are performed.
    %---
    z = InitialGuess(y,IsFinite);
        
else
    z = zeros(sizy);
end
%---
z0 = z;
y(~IsFinite) = 0; % arbitrary values for missing y-data
%---
tol = 1;
RobustIterativeProcess = true;
RobustStep = 1; %#ok<NASGU>
nit = 0;
%--- Error on p. Smoothness parameter s = 10^p
errp = 0.1;
opt = optimset('TolX',errp);
%--- Relaxation factor RF: to speedup convergence
RF = 1 + 0.75*isweighted;

%% Main iterative process
%---
while RobustIterativeProcess
    %--- "amount" of weights (see the function GCVscore)
    aow = sum(Wtot(:))/noe; % 0 < aow <= 1
    %---
    while tol>TolZ && nit<MaxIter
        nit = nit+1;
        DCTy = dctn(Wtot.*(y-z)+z);
        if isauto && ~rem(log2(nit),1)
            %---
            % The generalized cross-validation (GCV) method is used.
            % We seek the smoothing parameter s that minimizes the GCV
            % score i.e. s = Argmin(GCVscore)^.
            % Because this process is time-consuming, it is performed from
            % time to time (when nit is a power of 2)
            %---
            fminbnd(@gcv,log10(sMinBnd),log10(sMaxBnd),opt);
        end
        z = RF*idctn(Gamma.*DCTy) + (1-RF)*z;
        
        % if no weighted/missing data => tol=0 (no iteration)
        tol = isweighted*norm(z0(:)-z(:))/norm(z(:));
       
        z0 = z; % re-initialization
    end
    exitflag = nit<MaxIter;

    if isrobust %-- Robust Smoothing: iteratively re-weighted process
        %--- average leverage
        h = sqrt(1+16*s); h = sqrt(1+h)/sqrt(2)/h; h = h^N; %#ok<UNRCH>
        %--- take robust weights into account
        Wtot = W.*RobustWeights(y-z,IsFinite,h);
        %--- re-initialize for another iterative weighted process
        isweighted = true; tol = 1; nit = 0; 
        %---
        RobustStep = RobustStep+1;
        RobustIterativeProcess = RobustStep<4; % 3 robust steps are enough.
    else
        RobustIterativeProcess = false; % stop the whole process
    end
end

z = cast(z, 'like', y);

% %% Warning messages
% %---
% if nargout<3 && ~exitflag
%     warning('MATLAB:smoothn:MaxIter',...
%         ['Maximum number of iterations (' int2str(MaxIter) ') has ',...
%         'been exceeded. Increase MaxIter option or decrease TolZ value.'])
% end


%% GCV score
%---
function GCVscore = gcv(p)
    % Search the smoothing parameter s that minimizes the GCV score
    %---
    s = 10^p;
    Gamma = 1./(1+s*Lambda.^2);
    %--- RSS = Residual sum-of-squares
    if aow>0.9 % aow = 1 means that all of the data are equally weighted
        % very much faster: does not require any inverse DCT
        RSS = norm(DCTy(:).*(Gamma(:)-1))^2;
    else
        % take account of the weights to calculate RSS:
        yhat = idctn(Gamma.*DCTy);
        RSS = norm(sqrt(Wtot(IsFinite)).*(y(IsFinite)-yhat(IsFinite)))^2;
    end
    %---
    TrH = sum(Gamma(:));
    GCVscore = RSS/nof/(1-TrH/noe)^2;
end

end

%% Initial Guess with weighted/missing data
function z = InitialGuess(y,I)
    %-- nearest neighbor interpolation (in case of missing values)
    if any(~I(:))
        [~,L] = bwdist(I);
        z = y;
        z(~I) = y(L(~I));
    else
        z = y;
    end
    %-- coarse fast smoothing using one-tenth of the DCT coefficients
    siz = size(z);
    z = dctn(z);
    for k = 1:ndims(z)
        z(ceil(siz(k)/10)+1:end,:) = 0;
        z = reshape(z,circshift(siz,[0 1-k]));
        z = shiftdim(z,1);
    end
    z = idctn(z);
end


%% SUBFUNCTION idctn
function y = dctn(y)
% function [y,w] = dctn(y,w)
%%%% 5/10/2023: SIMPLIFIED FOR CODEGEN BY CARL J. LEGLEITER

%DCTN N-D discrete cosine transform.
%   Y = DCTN(X) returns the discrete cosine transform of X. The array Y is
%   the same size as X and contains the discrete cosine transform
%   coefficients. This transform can be inverted using IDCTN.
%
%   Class Support
%   -------------
%   Input array can be numeric or logical. The returned array is of class
%   double.
%
%   Reference
%   ---------
%   Narasimha M. et al, On the computation of the discrete cosine
%   transform, IEEE Trans Comm, 26, 6, 1978, pp 934-936.
%
%   Example
%   -------
%       RGB = imread('autumn.tif');
%       I = rgb2gray(RGB);
%       J = dctn(I);
%       imshow(log(abs(J)),[]), colormap(jet), colorbar
%
%   The commands below set values less than magnitude 10 in the DCT matrix
%   to zero, then reconstruct the image using the inverse DCT.
%
%       J(abs(J)<10) = 0;
%       K = idctn(J);
%       figure, imshow(I)
%       figure, imshow(K,[0 255])
%
%   See also IDCTN, DCT, DCT2.
%
%   -- Damien Garcia -- 2008/06, revised 2009/11


y = double(y);
sizy = size(y);
y = squeeze(y);

dimy = ndims(y);
if isvector(y)
    dimy = 1;
    y = y(:);
end

w = cell(1,dimy);
for dim = 1:dimy
    n = (dimy==1)*numel(y) + (dimy>1)*sizy(dim);
    w{dim} = exp(1i*(0:n-1)'*pi/2/n);
end

if ~isreal(y)
    y = complex(dctn(real(y),w),dctn(imag(y),w));
else
    for dim = 1:dimy
        siz = size(y);
        n = siz(1);
        y = y([1:2:n 2*floor(n/2):-2:2],:);
        y = reshape(y,n,[]);
        y = y*sqrt(2*n);
        y = ifft(y,[],1);
        y = bsxfun(@times,y,w{dim});
        y = real(y);
        y(1,:) = y(1,:)/sqrt(2);
        y = reshape(y,siz);
        y = shiftdim(y,1);
    end
end
        
y = reshape(y,sizy);
end % dctn function


%% SUBFUNCTION idctn
function y = idctn(y)
%%%% 5/10/2023: SIMPLIFIED FOR CODEGEN BY CARL J. LEGLEITER
% function [y,w] = idctn(y,w)

%IDCTN N-D inverse discrete cosine transform.
%   X = IDCTN(Y) inverts the N-D DCT transform, returning the original
%   array if Y was obtained using Y = DCTN(X).
%
%   Class Support
%   -------------
%   Input array can be numeric or logical. The returned array is of class
%   double.
%
%   Reference
%   ---------
%   Narasimha M. et al, On the computation of the discrete cosine
%   transform, IEEE Trans Comm, 26, 6, 1978, pp 934-936.
%
%   Example
%   -------
%       RGB = imread('autumn.tif');
%       I = rgb2gray(RGB);
%       J = dctn(I);
%       imshow(log(abs(J)),[]), colormap(jet), colorbar
%
%   The commands below set values less than magnitude 10 in the DCT matrix
%   to zero, then reconstruct the image using the inverse DCT.
%
%       J(abs(J)<10) = 0;
%       K = idctn(J);
%       figure, imshow(I)
%       figure, imshow(K,[0 255])
%
%   See also DCTN, IDCT, IDCT2.
%
%   -- Damien Garcia -- 2009/04, revised 2009/11


y = double(y);
sizy = size(y);
y = squeeze(y);

dimy = ndims(y);
if isvector(y)
    dimy = 1;
    y = y(:);
end

w = cell(1,dimy);
for dim = 1:dimy
    n = (dimy==1)*numel(y) + (dimy>1)*sizy(dim);
    w{dim} = exp(1i*(0:n-1)'*pi/2/n);
end

if ~isreal(y)
    y = complex(idctn(real(y),w),idctn(imag(y),w));
else
        for dim = 1:dimy
            siz = size(y);
            n = siz(1);
            y = reshape(y,n,[]);
            y = bsxfun(@times,y,w{dim});
            y(1,:) = y(1,:)/sqrt(2);
            y = ifft(y,[],1);
            y = real(y*sqrt(2*n));
            I = (1:n)*0.5+0.5;
            I(2:2:end) = n-I(1:2:end-1)+1;
            y = y(I,:);
            y = reshape(y,siz);
            y = shiftdim(y,1);            
        end
end
        
y = reshape(y,sizy);

end % idctn function


function B=inpaint_nans(A,method)

% Author: John D'Errico
% e-mail address: woodchips@rochester.rr.com
% Release: 2
% Release date: 4/15/06

% INPAINT_NANS: in-paints over nans in an array
% usage: B=INPAINT_NANS(A)          % default method
% usage: B=INPAINT_NANS(A,method)   % specify method used
%
% Solves approximation to one of several pdes to
% interpolate and extrapolate holes in an array
%
% arguments (input):
%   A - nxm array with some NaNs to be filled in
%
%   method - (OPTIONAL) scalar numeric flag - specifies
%       which approach (or physical metaphor to use
%       for the interpolation.) All methods are capable
%       of extrapolation, some are better than others.
%       There are also speed differences, as well as
%       accuracy differences for smooth surfaces.
%
%       methods {0,1,2} use a simple plate metaphor.
%       method  3 uses a better plate equation,
%                 but may be much slower and uses
%                 more memory.
%       method  4 uses a spring metaphor.
%       method  5 is an 8 neighbor average, with no
%                 rationale behind it compared to the
%                 other methods. I do not recommend
%                 its use.
%
%       method == 0 --> (DEFAULT) see method 1, but
%         this method does not build as large of a
%         linear system in the case of only a few
%         NaNs in a large array.
%         Extrapolation behavior is linear.
%         
%       method == 1 --> simple approach, applies del^2
%         over the entire array, then drops those parts
%         of the array which do not have any contact with
%         NaNs. Uses a least squares approach, but it
%         does not modify known values.
%         In the case of small arrays, this method is
%         quite fast as it does very little extra work.
%         Extrapolation behavior is linear.
%         
%       method == 2 --> uses del^2, but solving a direct
%         linear system of equations for nan elements.
%         This method will be the fastest possible for
%         large systems since it uses the sparsest
%         possible system of equations. Not a least
%         squares approach, so it may be least robust
%         to noise on the boundaries of any holes.
%         This method will also be least able to
%         interpolate accurately for smooth surfaces.
%         Extrapolation behavior is linear.
%         
%       method == 3 --+ See method 0, but uses del^4 for
%         the interpolating operator. This may result
%         in more accurate interpolations, at some cost
%         in speed.
%         
%       method == 4 --+ Uses a spring metaphor. Assumes
%         springs (with a nominal length of zero)
%         connect each node with every neighbor
%         (horizontally, vertically and diagonally)
%         Since each node tries to be like its neighbors,
%         extrapolation is as a constant function where
%         this is consistent with the neighboring nodes.
%
%       method == 5 --+ See method 2, but use an average
%         of the 8 nearest neighbors to any element.
%         This method is NOT recommended for use.
%
%
% arguments (output):
%   B - nxm array with NaNs replaced
%
%
% Example:
%  [x,y] = meshgrid(0:.01:1);
%  z0 = exp(x+y);
%  znan = z0;
%  znan(20:50,40:70) = NaN;
%  znan(30:90,5:10) = NaN;
%  znan(70:75,40:90) = NaN;
%
%  z = inpaint_nans(znan);
%
%
% See also: griddata, interp1
%
% Author: John D'Errico
% e-mail address: woodchips@rochester.rr.com
% Release: 2
% Release date: 4/15/06


% I always need to know which elements are NaN,
% and what size the array is for any method
[n,m]=size(A);
A=A(:);
nm=n*m;
k=isnan(A(:));

% list the nodes which are known, and which will
% be interpolated
nan_list=find(k);
known_list=find(~k);

% how many nans overall
nan_count=length(nan_list);

% convert NaN indices to (r,c) form
% nan_list==find(k) are the unrolled (linear) indices
% (row,column) form
[nr,nc]=ind2sub([n,m],nan_list);

% both forms of index in one array:
% column 1 == unrolled index
% column 2 == row index
% column 3 == column index
nan_list=[nan_list,nr,nc];

% supply default method
if (nargin<2) || isempty(method)
  method = 0;
elseif ~ismember(method,0:5)
  error 'If supplied, method must be one of: {0,1,2,3,4,5}.'
end

% for different methods
switch method
 case 0
  % The same as method == 1, except only work on those
  % elements which are NaN, or at least touch a NaN.
  
  % horizontal and vertical neighbors only
  talks_to = [-1 0;0 -1;1 0;0 1];
  neighbors_list=identify_neighbors(n,m,nan_list,talks_to);
  
  % list of all nodes we have identified
  all_list=[nan_list;neighbors_list];
  
  % generate sparse array with second partials on row
  % variable for each element in either list, but only
  % for those nodes which have a row index > 1 or < n
  L = find((all_list(:,2) > 1) & (all_list(:,2) < n)); 
  nl=length(L);
  if nl>0
    fda=sparse(repmat(all_list(L,1),1,3), ...
      repmat(all_list(L,1),1,3)+repmat([-1 0 1],nl,1), ...
      repmat([1 -2 1],nl,1),nm,nm);
  else
    fda=spalloc(n*m,n*m,size(all_list,1)*5);
  end
  
  % 2nd partials on column index
  L = find((all_list(:,3) > 1) & (all_list(:,3) < m)); 
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(all_list(L,1),1,3), ...
      repmat(all_list(L,1),1,3)+repmat([-n 0 n],nl,1), ...
      repmat([1 -2 1],nl,1),nm,nm);
  end
  
  % eliminate knowns
  rhs=-fda(:,known_list)*double(A(known_list));
  k=find(any(fda(:,nan_list(:,1)),2));
  
  % and solve...
  B=A;
  B(nan_list(:,1))=fda(k,nan_list(:,1))\rhs(k);
  
 case 1
  % least squares approach with del^2. Build system
  % for every array element as an unknown, and then
  % eliminate those which are knowns.

  % Build sparse matrix approximating del^2 for
  % every element in A.
  % Compute finite difference for second partials
  % on row variable first
  [i,j]=ndgrid(2:(n-1),1:m);
  ind=i(:)+(j(:)-1)*n;
  np=(n-2)*m;
  fda=sparse(repmat(ind,1,3),[ind-1,ind,ind+1], ...
      repmat([1 -2 1],np,1),n*m,n*m);
  
  % now second partials on column variable
  [i,j]=ndgrid(1:n,2:(m-1));
  ind=i(:)+(j(:)-1)*n;
  np=n*(m-2);
  fda=fda+sparse(repmat(ind,1,3),[ind-n,ind,ind+n], ...
      repmat([1 -2 1],np,1),nm,nm);
  
  % eliminate knowns
  rhs=-fda(:,known_list)*double(A(known_list));
  k=find(any(fda(:,nan_list),2));
  
  % and solve...
  B=A;
  B(nan_list(:,1))=fda(k,nan_list(:,1))\rhs(k);
  
 case 2
  % Direct solve for del^2 BVP across holes

  % generate sparse array with second partials on row
  % variable for each nan element, only for those nodes
  % which have a row index > 1 or < n
  L = find((nan_list(:,2) > 1) & (nan_list(:,2) < n)); 
  nl=length(L);
  if nl>0
    fda=sparse(repmat(nan_list(L,1),1,3), ...
      repmat(nan_list(L,1),1,3)+repmat([-1 0 1],nl,1), ...
      repmat([1 -2 1],nl,1),n*m,n*m);
  else
    fda=spalloc(n*m,n*m,size(nan_list,1)*5);
  end
  
  % 2nd partials on column index
  L = find((nan_list(:,3) > 1) & (nan_list(:,3) < m)); 
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,3), ...
      repmat(nan_list(L,1),1,3)+repmat([-n 0 n],nl,1), ...
      repmat([1 -2 1],nl,1),n*m,n*m);
  end
  
  % fix boundary conditions at extreme corners
  % of the array in case there were nans there
  if ismember(1,nan_list(:,1))
    fda(1,[1 2 n+1])=[-2 1 1];
  end
  if ismember(n,nan_list(:,1))
    fda(n,[n, n-1,n+n])=[-2 1 1];
  end
  if ismember(nm-n+1,nan_list(:,1))
    fda(nm-n+1,[nm-n+1,nm-n+2,nm-n])=[-2 1 1];
  end
  if ismember(nm,nan_list(:,1))
    fda(nm,[nm,nm-1,nm-n])=[-2 1 1];
  end
  
  % eliminate knowns
  rhs=-fda(:,known_list)*double(A(known_list));
  
  % and solve...
  B=A;
  k=nan_list(:,1);
  B(k)=fda(k,k)\rhs(k);
  
 case 3
  % The same as method == 0, except uses del^4 as the
  % interpolating operator.
  
  % del^4 template of neighbors
  talks_to = [-2 0;-1 -1;-1 0;-1 1;0 -2;0 -1; ...
      0 1;0 2;1 -1;1 0;1 1;2 0];
  neighbors_list=identify_neighbors(n,m,nan_list,talks_to);
  
  % list of all nodes we have identified
  all_list=[nan_list;neighbors_list];
  
  % generate sparse array with del^4, but only
  % for those nodes which have a row & column index
  % >= 3 or <= n-2
  L = find( (all_list(:,2) >= 3) & ...
            (all_list(:,2) <= (n-2)) & ...
            (all_list(:,3) >= 3) & ...
            (all_list(:,3) <= (m-2)));
  nl=length(L);
  if nl>0
    % do the entire template at once
    fda=sparse(repmat(all_list(L,1),1,13), ...
        repmat(all_list(L,1),1,13) + ...
        repmat([-2*n,-n-1,-n,-n+1,-2,-1,0,1,2,n-1,n,n+1,2*n],nl,1), ...
        repmat([1 2 -8 2 1 -8 20 -8 1 2 -8 2 1],nl,1),nm,nm);
  else
    fda=spalloc(n*m,n*m,size(all_list,1)*5);
  end
  
  % on the boundaries, reduce the order around the edges
  L = find((((all_list(:,2) == 2) | ...
             (all_list(:,2) == (n-1))) & ...
            (all_list(:,3) >= 2) & ...
            (all_list(:,3) <= (m-1))) | ...
           (((all_list(:,3) == 2) | ...
             (all_list(:,3) == (m-1))) & ...
            (all_list(:,2) >= 2) & ...
            (all_list(:,2) <= (n-1))));
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(all_list(L,1),1,5), ...
      repmat(all_list(L,1),1,5) + ...
        repmat([-n,-1,0,+1,n],nl,1), ...
      repmat([1 1 -4 1 1],nl,1),nm,nm);
  end
  
  L = find( ((all_list(:,2) == 1) | ...
             (all_list(:,2) == n)) & ...
            (all_list(:,3) >= 2) & ...
            (all_list(:,3) <= (m-1)));
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(all_list(L,1),1,3), ...
      repmat(all_list(L,1),1,3) + ...
        repmat([-n,0,n],nl,1), ...
      repmat([1 -2 1],nl,1),nm,nm);
  end
  
  L = find( ((all_list(:,3) == 1) | ...
             (all_list(:,3) == m)) & ...
            (all_list(:,2) >= 2) & ...
            (all_list(:,2) <= (n-1)));
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(all_list(L,1),1,3), ...
      repmat(all_list(L,1),1,3) + ...
        repmat([-1,0,1],nl,1), ...
      repmat([1 -2 1],nl,1),nm,nm);
  end
  
  % eliminate knowns
  rhs=-fda(:,known_list)*double(A(known_list));
  k=find(any(fda(:,nan_list(:,1)),2));
  
  % and solve...
  B=A;
  B(nan_list(:,1))=fda(k,nan_list(:,1))\rhs(k);
  
 case 4
  % Spring analogy
  % interpolating operator.
  
  % list of all springs between a node and a horizontal
  % or vertical neighbor
  hv_list=[-1 -1 0;1 1 0;-n 0 -1;n 0 1];
  hv_springs=[];
  for i=1:4
    hvs=nan_list+repmat(hv_list(i,:),nan_count,1);
    k=(hvs(:,2)>=1) & (hvs(:,2)<=n) & (hvs(:,3)>=1) & (hvs(:,3)<=m);
    hv_springs=[hv_springs;[nan_list(k,1),hvs(k,1)]]; %#ok<AGROW>
  end

  % delete replicate springs
  hv_springs=unique(sort(hv_springs,2),'rows');
  
  % build sparse matrix of connections, springs
  % connecting diagonal neighbors are weaker than
  % the horizontal and vertical springs
  nhv=size(hv_springs,1);
  springs=sparse(repmat((1:nhv)',1,2),hv_springs, ...
     repmat([1 -1],nhv,1),nhv,nm);
  
  % eliminate knowns
  rhs=-springs(:,known_list)*double(A(known_list));
  
  % and solve...
  B=A;
  B(nan_list(:,1))=springs(:,nan_list(:,1))\rhs;
  
 case 5
  % Average of 8 nearest neighbors
  
  % generate sparse array to average 8 nearest neighbors
  % for each nan element, be careful around edges
  fda=spalloc(n*m,n*m,size(nan_list,1)*9);
  
  % -1,-1
  L = find((nan_list(:,2) > 1) & (nan_list(:,3) > 1)); 
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([-n-1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end
  
  % 0,-1
  L = find(nan_list(:,3) > 1);
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([-n, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end

  % +1,-1
  L = find((nan_list(:,2) < n) & (nan_list(:,3) > 1));
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([-n+1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end

  % -1,0
  L = find(nan_list(:,2) > 1);
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([-1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end

  % +1,0
  L = find(nan_list(:,2) < n);
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end

  % -1,+1
  L = find((nan_list(:,2) > 1) & (nan_list(:,3) < m)); 
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([n-1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end
  
  % 0,+1
  L = find(nan_list(:,3) < m);
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([n, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end

  % +1,+1
  L = find((nan_list(:,2) < n) & (nan_list(:,3) < m));
  nl=length(L);
  if nl>0
    fda=fda+sparse(repmat(nan_list(L,1),1,2), ...
      repmat(nan_list(L,1),1,2)+repmat([n+1, 0],nl,1), ...
      repmat([1 -1],nl,1),n*m,n*m);
  end
  
  % eliminate knowns
  rhs=-fda(:,known_list)*double(A(known_list));
  
  % and solve...
  B=A;
  k=nan_list(:,1);
  B(k)=fda(k,k)\rhs(k);
  
end

% all done, make sure that B is the same shape as
% A was when we came in.
B=reshape(B,n,m);

% ====================================================
%      end of main function
% ====================================================
end % inpaint_nans function

% ====================================================
%      begin subfunctions
% ====================================================
function neighbors_list=identify_neighbors(n,m,nan_list,talks_to)
% identify_neighbors: identifies all the neighbors of
%   those nodes in nan_list, not including the nans
%   themselves
%
% arguments (input):
%  n,m - scalar - [n,m]=size(A), where A is the
%      array to be interpolated
%  nan_list - array - list of every nan element in A
%      nan_list(i,1) == linear index of i'th nan element
%      nan_list(i,2) == row index of i'th nan element
%      nan_list(i,3) == column index of i'th nan element
%  talks_to - px2 array - defines which nodes communicate
%      with each other, i.e., which nodes are neighbors.
%
%      talks_to(i,1) - defines the offset in the row
%                      dimension of a neighbor
%      talks_to(i,2) - defines the offset in the column
%                      dimension of a neighbor
%      
%      For example, talks_to = [-1 0;0 -1;1 0;0 1]
%      means that each node talks only to its immediate
%      neighbors horizontally and vertically.
% 
% arguments(output):
%  neighbors_list - array - list of all neighbors of
%      all the nodes in nan_list

if ~isempty(nan_list)
  % use the definition of a neighbor in talks_to
  nan_count=size(nan_list,1);
  talk_count=size(talks_to,1);
  
  nn=zeros(nan_count*talk_count,2);
  j=[1,nan_count];
  for i=1:talk_count
    nn(j(1):j(2),:)=nan_list(:,2:3) + ...
        repmat(talks_to(i,:),nan_count,1);
    j=j+nan_count;
  end
  
  % drop those nodes which fall outside the bounds of the
  % original array
  L = (nn(:,1)<1)|(nn(:,1)>n)|(nn(:,2)<1)|(nn(:,2)>m); 
  nn(L,:)=[];
  
  % form the same format 3 column array as nan_list
  neighbors_list=[sub2ind([n,m],nn(:,1),nn(:,2)),nn];
  
  % delete replicates in the neighbors list
  neighbors_list=unique(neighbors_list,'rows');
  
  % and delete those which are also in the list of NaNs.
  neighbors_list=setdiff(neighbors_list,nan_list,'rows');
  
else
  neighbors_list=[];
end

end % identify_neighbors subfunction of inpaint_nans

end % main function