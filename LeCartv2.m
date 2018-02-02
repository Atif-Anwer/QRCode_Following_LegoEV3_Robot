%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% QR Code Following Cart
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed by: Atif Anwer
% 
% Hardware requirement:
%     1x LEGO EV3
%     2x Lego Large Motors (EV3)
%     1x Lego Ultrasonic Transeiver (EV3)
%     1x Webcam (Logitech C270 used)
%     1x Arduino MEGAADK
%     5x HC-SR04 Ultrasonic Sensors
% 
% Basic Code flow:
%     1. Get Deviation from center and Direction from KLT tracking
%         1(a). Find Region of Interest and its center (4x Red Circles in a square) 
%         1(b). Find SURF Feature points (QR Code with 4x Red Circles at the corners)
%         1(c). Match SURF features with QR Code template in file
%         1(d). Continue if QR code found else stop moving
%     2. Get Values of 5x Ultrasonic sensors from Arduino
%     3. Get value from 1x front LEGO ultrasonic sensor
%     4. Calculate Direction to turn based on the position of QR code in the field of view
%     5. Calculate speed of turn for both motors using PID feedback control
% 	6. Save values to pass on to cartTurn function to execute the values
%     7. Check if the main window is open. If its closed; exit the program.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LeCartv2 ()
	clc;
	close all;

	global cam;
	global FrontLegoUltrasonic;
	global legoEV3;
	global pointTracker;
% 	global serialPort;
% 	global SR04Value;
	global videoPlayer;
	global distanceFromCenter;						% Final calculated Distance of QR code from Image Center
	global leftMotor;
	global rightMotor;

	global Kp;                          			% Proportional
	global Ki;                          			% Integral
	global Kd;                          			% Derivative
	global TargetSpeed;                 			% Target Motor Power
	global integral;                    			% the place where we will store our integral
	global lastError;                   			% the place where we will store the last error value
	global derivative;                  			% the place where we will store the derivative

	global FLAG_moveStraight;
	global FLAG_noObstacleDetected;
	global FLAG_runloop;
	global FLAG_Stop;
	global FLAG_moveLeft;
	global FLAG_moveRight

% 	global handle;
% 	global xAxis;
	global index;
	global targetQRImg;
	global targetQRpoints;
	global QRFeatures;
	global QRPoints;
    global AnimatedLineFigure;
    global safeDistance;

%     handle = animatedline('Marker','o');
%     axis([1,150,-100,100]);
%     xAxis = linspace(1,150,150);
	index = 1;

	% Center width of no turn -  go straight function (in Pixels)
	global centerWidthAllowed;

	Kp = 1;                             	% Orig = 2.4; Kp is multiplied - Must not exceed target power
	Ki = 0.01;                           	% REMEMBER we are using Ki*100 so this is really 1 !
	Kd = 0.01;                             	% REMEMBER we are using Kd*100 so this is really 100!

	% Center width allowed is 30px or 5% of TargetSpeed
	centerWidthAllowed = 30;

	safeDistance = 0.6;             		% Distance for ultrasonic
	TargetSpeed = 20;						% Target Motor Power
	integral = 0;                       	% the place where we will store our integral
	lastError = 0;                      	% the place where we will store the last error value
	derivative = 0;                     	% the place where we will store the derivative code
	% initialize vairables
	initAll ();

	 % run while window is open - else close and cleanup
	FLAG_runloop = true;
	FLAG_moveStraight = true;  				% false so that first loop starts the motor
	FLAG_noObstacleDetected = true;
	FLAG_Stop = false;
	firstTry = true;

	% Initialize a Tracker to Track the Points  With the feature points identified, you can now use the
	% |vision.PointTracker| System object to track them. For each point in the
	% previous frame, the point tracker attempts to find the corresponding point in the current frame.
	pointTracker = vision.PointTracker('MaxIterations', 30, 'BlockSize', [31,31], 'MaxBidirectionalError', 2);

	targetQRImg = rgb2gray(targetQRImg);
	targetQRpoints = detectSURFFeatures(targetQRImg);
	targetQRpoints = selectStrongest(targetQRpoints, 100);
	[QRFeatures, QRPoints] = extractFeatures(targetQRImg, targetQRpoints);

	% run the loop forever
	while FLAG_runloop
		try
			
			% track the QR code and get error distance
			distanceFromCenter  = KLTtracking ();
			% The distance error we get is in Pixels. Before implementing
			% any control scheme; we need to convert it to Lego Motor Power
			% limits which is ?TargetSpeed. Image width is 640; center will be 320.
			distanceFromCenter = round (distanceFromCenter / 320 * TargetSpeed);
			fprintf('\n\n distanceFromCenter = %d', distanceFromCenter);

			if distanceFromCenter < centerWidthAllowed
				FLAG_moveLeft = true;
			elseif distanceFromCenter > centerWidthAllowed
				FLAG_moveRight = true;
			end

			LegoUSonic = GetSR04Values (FrontLegoUltrasonic);
			fprintf('\n FrontLegoUltrasonic = %d', round(LegoUSonic));
			if LegoUSonic < safeDistance ;
				% raise flags that cart has stopped
				FLAG_moveStraight = false;
				FLAG_Stop = true;
				cartStop();
			else
				FLAG_Stop = false;
			end

			[lastError, motorADir, motorBDir, powerA, powerD] = calculateTurnDirDistandSpeed (distanceFromCenter,lastError, LegoUSonic);
			if firstTry == true || FLAG_Stop == false;
				start(leftMotor);                            % Start motor
				start(rightMotor);
				firstTry = false;
				cartTurn(motorADir, motorBDir, powerA, powerD, FLAG_Stop);
			end
			% Run while videoplayter is open
			FLAG_runloop = isOpen(videoPlayer);
        catch ERROR
			% do nothing
%             clf(AnimatedLineFigure,'reset')
			LegoUSonic = GetSR04Values (FrontLegoUltrasonic);
			fprintf('\n Catch Error (Main Loop)');
			if LegoUSonic < safeDistance || FLAG_Stop == true
				% raise flags that cart has stopped
				FLAG_moveStraight = false;
				FLAG_Stop = true;
				cartStop();
			else
				FLAG_Stop = false;
			end

			% Run while videoplayter is open
			FLAG_runloop = isOpen(videoPlayer);

		end
	end
	% cleanup
	cartStop();
	delete (cam);
	delete(legoEV3);
	release(videoPlayer);
	release(pointTracker);
	close all;
end

%% Get the 5x ultrasonic sensor values and decide if any object is in front or sides
function [LegoUSonic] = GetSR04Values (~)

% 	global FLAG_moveStraight;
% 	global FLAG_rightTurnAllowed;
% 	global FLAG_leftTurnAllowed;
% 	global SR04Value;
% 	global serialPort;
% 	global FLAG_noObstacleDetected;
	global FrontLegoUltrasonic;
    global safeDistance;

	LegoUSonic = readDistance(FrontLegoUltrasonic);
    if LegoUSonic < safeDistance
        % if object in front
        figure(99);
        subplot(3,2,[1 2]);
        set(gca,'color','r');
        return;         % stop immediately
    else
        % plot the ultrasonic response
		plotVisually();
    end

% 	% get values from the 5x sensors
% 	fprintf(serialPort,'R');
%
% 	% Data will get values sperated by spaces, ending with a EOL char
% 	data = fscanf(serialPort);
%
% 	% split the string into values, convert to number array
% 	% 	 Get 1st value, split the remainder, get val from remainder
% 	%	 untill all 5 sensor values are read from the string
% 	[str,rem] = strtok(data);
% 	SR04Value(1) = real(str2double(str));
% 	for i = 2:5
% 		SR04Value(i) = real(str2double(strtok(rem)));
% 	end
%
% 	if FLAG_noObstacleDetected == false;
% 		% all clear
% 		FLAG_moveStraight = true;
% 		FLAG_leftTurnAllowed = true;
% 		FLAG_rightTurnAllowed = true;
%
% 		% plot the ultrasonic response
% 		plotVisually();
% 		% reset flag
% 		FLAG_noObstacleDetected = true;
% 	end
%
% 	% display on the UI
% 	if SR04Value(3) <= 30
% 		% if object in front
% 		FLAG_moveStraight = false;
% 		FLAG_Stop = true;
% 		figure(99);
% 		subplot(3,2,[1 2]);
% 		set(gca,'color','r');
% 		return;         % stop immediately
% 	elseif SR04Value(1) <= 60
% 		% if object on left side
% 		FLAG_leftTurnAllowed = false;
% 		figure(99);
% 		subplot(3,2,5);
% 		set(gca,'color','r');
% 		FLAG_noObstacleDetected = false;
% 	elseif SR04Value(2) <= 60
% 		% if object on left side
% 		FLAG_leftTurnAllowed = false;
% 		figure(99);
% 		subplot(3,2,3);
% 		set(gca,'color','r');
% 		FLAG_noObstacleDetected = false;
% 	elseif SR04Value(4) <= 60
% 		% if object on right side
% 		FLAG_rightTurnAllowed = false;
% 		figure(99);
% 		subplot(3,2,4);
% 		set(gca,'color','r');
% 		FLAG_noObstacleDetected = false;
% 	elseif SR04Value(5) <= 60
% 		% if object on right side
% 		FLAG_rightTurnAllowed = false;
% 		figure(99);
% 		subplot(3,2,6);
% 		set(gca,'color','r');
% 		FLAG_noObstacleDetected = false;
% 	end
%
% 	% Force update the figure
% 	%     drawnow limitrate;
	drawnow;
end

%%
function [distanceFromCenter ] = KLTtracking ()
% Init global vars
	global cam;
	global videoPlayer;
	global FLAG_runloop;
	global numPts;
	global frameCount;
	global pointTracker;
	global videoFrame;
	global oldPoints;
	global bboxPolygon;
	global oldInliers;
	global points;
	global w;
	global h;

	global centerWidthAllowed;
	global FLAG_Stop;

	global legoEV3;

	global FLAG_redCircles;

	global targetQRImg;
	global targetQRpoints;
	global QRFeatures;
	global QRPoints;
    
    global PIDTarget;
    global PIDspeedA;
    global PIDspeedD;

	% code
	hold on;
	try
		% Take the first image from the cam
		videoFrame = snapshot(cam);
		videoFrameGrey = rgb2gray(videoFrame);
		frameCount = frameCount + 1;

		if numPts < 10 || frameCount == 150
			% if frameCount == 150 or No surf points or coming first time
			% reset frame count
			frameCount = 0;
            clearpoints(PIDTarget);
            clearpoints(PIDspeedA);
            clearpoints(PIDspeedD);
			% Detection mode.: run SURF re-track the object after every 10
			% secs
			% while (1)
			% for mm = 1 : 1

				% videoFrame = snapshot(cam);
				% videoFrameGrey = rgb2gray(videoFrame);
				[x1, y1, x2, y2, w, h, ~, centers, ~,  centerBox_X, centerBox_Y] = detect_circles(frameCount,videoFrameGrey, videoFrame);
				% only quit if 4 centers are found
				% Also; w/h or h/w define aspect ratio of the QR code  circles detected. To remove stray circles detected,
				% we define aspect ratio as a check to find the  required rectangle.
				% 				fprintf('\n W/H = %2.2f, H/W = %2.2f', w/h, h/w);

				% 				if length(centers) == 4 && w > 0 && h > 0 && (w/h <=1.3 && w/h >=0.7) || (h/w <=1.3 && h/w >=0.7)
				if FLAG_redCircles == true
					centerBox = round([centerBox_X centerBox_Y]);
% 					distanceFromCenter = (centerBox(1)-size(videoFrame, 2)/2);
					bboxPolygon = [x1, y1, x1+w, y1, x1+w, y1+h, x1, y1+h];
					videoFrame = insertMarker(videoFrame, centerBox, 'o', 'Color', 'red', 'Size', 5);
					hold on;
					% box found so start the motor
					% FLAG_Stop = false;
% 					break;
				else
					% box not found so stop the motor
					FLAG_Stop = true;
% 					continue;
				end

			% detect SURF features
			points = detectSURFFeatures(videoFrameGrey, 'ROI', [x1 y1 w h]);

			[tgtFeatures, tgtPoints] = extractFeatures(videoFrameGrey, points);
			featurePairs = matchFeatures(QRFeatures, tgtFeatures);
			matchedQRPoints = QRPoints(featurePairs(:, 1), :);
			matchedScenePoints = tgtPoints(featurePairs(:, 2), :);
			if isempty(matchedQRPoints) || isempty(matchedScenePoints)
				% if no matched points then return
				FLAG_Stop = true;
				return;
			end
			[tform, inlierBoxPoints, inlierScenePoints] = estimateGeometricTransform(...
				matchedQRPoints, matchedScenePoints, 'affine');

%             figure;
%             showMatchedFeatures(targetQRImg, videoFrameGrey, inlierBoxPoints, ...
%                 inlierScenePoints, 'montage');
%             title('Matched Points (Inliers Only)');

			if (points.Count ~= 0)
				% if circles / QR Code found..
				numPts = points.Count;
				release(pointTracker);
				initialize(pointTracker,points.Location,videoFrame);
				%                 bboxPolygon = [x1, y1, x1+w, y1, x1+w, y1+h, x1, y1+h];
				videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon);
				% Make a copy of the points to be used for computing the geometric
				% % transformation between the points in the previous and the current frames
				oldPoints = points.Location;
			end
		else
			% Tracking mode.
			% Track the points. Note that some points may be lost.
			[points, isFound] = step(pointTracker, videoFrame);
			% display of all the correctly tracked featrures
			visiblePoints = points(isFound, :);
			oldInliers = oldPoints(isFound, :);
			numPts = size(visiblePoints, 1);

			if numPts >= 10
				% Estimate the geometric transformation between the old points
				% and the new points and eliminate outliers

				% draw previous bbox
				videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'Color', 'red');
				hold on;

				% The |estimateGeometricTransform| function is used to estimate  the translation, rotation, and scale between the old points and the  new points. This transformation is applied to the bounding box
				% bidirectional error constraint to make it more robust in the presence  of noise and clutter.
				[xform, ~, visiblePoints] = estimateGeometricTransform(...
						oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

				% Apply the transformation to the bounding box
				[bboxPolygon(1:2:end), bboxPolygon(2:2:end)] ...
					= transformPointsForward(xform, bboxPolygon(1:2:end), bboxPolygon(2:2:end));

				% Apply the transformation to the bounding box.
				% bboxPoints = transformPointsForward(xform, bboxPoints);

				% Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
				% format required by insertShape.
				bboxPolygon = reshape(bboxPolygon', 1, []);
				% Insert a bounding box around the object being tracked
				videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon);

				% Display tracked points
				videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'yellow');
				hold on;

				% Reset the points.
				oldPoints = visiblePoints;
				setPoints(pointTracker, oldPoints);

				lowerleftCorner = min(oldPoints);
				upperrightCorner = max(oldPoints);
				centerBox = lowerleftCorner/2 + upperrightCorner/2;
				videoFrame = insertMarker(videoFrame, centerBox, 'o', 'Color', 'red', 'Size', 5);
				hold on;

				distanceFromCenter = (centerBox(1)-size(videoFrame, 2)/2);
				% centerWidthAllowed = 30;
				if  distanceFromCenter > centerWidthAllowed
					position = [size(videoFrame, 2)/2 10 220 1];
					FLAG_Stop = false;
					labelString = ['>>>>> MOVING RIGHT: ', num2str(floor(centerBox(1)-size(videoFrame, 2)/2)), 'px'];
				elseif distanceFromCenter < -centerWidthAllowed
					FLAG_Stop = false;
					position = [size(videoFrame, 2)/2-220 10 220 1];
					labelString = [num2str(floor(centerBox(1)-size(videoFrame, 2)/2)), 'px', ': MOVING LEFT <<<<<< ', ];
				else
					position = [size(videoFrame, 2)/2-60 10 120 1];
					FLAG_Stop = false;
					labelString = ('>>> Center <<<');
				end

				writeStatusLight(legoEV3,'green','solid')
				videoFrame = insertObjectAnnotation(videoFrame,'rectangle',position,labelString,'LineWidth',3,'Color','cyan','TextColor','black');

				tformedPointsX = bboxPolygon(1:2:end);
				tformedPointsY = bboxPolygon(2:2:end);
				% points returned:
				% p1 - p2
				% |    |
				% p4 - p3

				% points required:
				% p1 - p2
				% |    |
				% p3 - p4

				% Error correction to cover for Negative or over extended points
				if tformedPointsX(1) <= 0
					x1 = 1;
				else
					x1 = tformedPointsX(1);
				end

				if tformedPointsY(1) <= 0
					y1 = 1;
				else
					y1 = tformedPointsY(1);
				end

				if tformedPointsX(3) <= size(videoFrame, 2)
					x4 = tformedPointsX(3);
				else
					x4 = size(videoFrame, 2);
				end

				if tformedPointsY(3) <= size(videoFrame, 1)
					y4 = tformedPointsY(3);
				else
					y4 = size(videoFrame, 1);
				end

				w = x4 - x1;
				h = y4 - y1;
            end
		end
		% Display the annotated video frame using the video player object.
		step(videoPlayer, videoFrame);
		% Check whether the video player window has been closed.
		FLAG_runloop = isOpen(videoPlayer);
	catch
		hold on;
		FLAG_Stop = true;
		position = [size(videoFrame, 2)/2-60 10 120 1];
		labelString = '>>>!!! LOST TRACK !!!<<<';
		videoFrame = insertObjectAnnotation(videoFrame,'rectangle',position,labelString,'LineWidth',3,'Color','red','TextColor','black');
		step(videoPlayer, videoFrame);
		fprintf('\n\n Catch Error (KLT Loop)');
		writeStatusLight(legoEV3,'orange','solid');
	end
end

%%
function [x1, y1, x2, y2, w, h, hx, centersStrong4, radiiStrong4, centerBox_X, centerBox_Y] = detect_circles(frameCount,grayImg, videoFrame)
		% detect QR Code corner circles
		% Requires grayscale image as input
		% returns [centers, radii] of the circles

		global FLAG_redCircles;

		[centers, radii] = imfindcircles(grayImg,[15 90],'ObjectPolarity','dark', ...
			'Sensitivity',0.95);

		% -------------------------
		% Create a logical image of a circle with specified
		% diameter, center, and image size.
		% First create the image.
		imageSizeX = 640;
		imageSizeY = 480;
		[columnsInImage, rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY);

		% Select Top4
		centersStrong4 = centers(1:4,:);
		radiiStrong4 = radii(1:4);

		% Next create the circle in the image.
		q=1;
		redIndex(length(centersStrong4)) = 0;
		for a = 1:length(centersStrong4)
			centerX = centersStrong4(a,1);
			centerY = centersStrong4(a,2);
			radius = radiiStrong4(a);
			circlePixels = (rowsInImage - centerY).^2 ...
				+ (columnsInImage - centerX).^2 <= radius.^2;
			% circlePixels is a 2D "logical" array.
			colormap([0 0 0; 1 1 1]);
			 % Extract the individual red, green, and blue color channels.
			redChannel = videoFrame(:, :, 1);
			greenChannel = videoFrame(:, :, 2);
			blueChannel = videoFrame(:, :, 3);
			% Get pixels inside mask for each color channel
			redPixels = redChannel(circlePixels);
			greenPixels = greenChannel(circlePixels);
			bluePixels = blueChannel(circlePixels);
			% Get means
			redMean = mean(redPixels);
			greenMean = mean(greenPixels);
			blueMean = mean(bluePixels);

			if redMean > 100 && greenMean < 110 && blueMean < 110
				redIndex(q) = a;
				q = q+1;
			end
		end
		% -------------------------

		figure(3);
		set(3, 'Position', [40 480 400 350] );
		imshow(grayImg);
		hold on;
		% plot rectangle
		Xcol = centersStrong4(:,1);
		Ycol = centersStrong4(:,2);
		% calculate order in which they from a convex hull / boundary
		boundaryOrder = boundary(Xcol,Ycol);
		plot(Xcol(boundaryOrder),Ycol(boundaryOrder),'r-',Xcol,Ycol,'b*');
		% -------------------------

		% Only return box if number of RED circles is 4
		if length(nonzeros(redIndex)) == 4
			% draw red circles
			hx = viscircles(centersStrong4, radiiStrong4 , 'Color','r');
		   % return all values otherwise catch error at output argument
			x1 = min(centersStrong4(:,1));
			y1 = min(centersStrong4(:,2));
			x2 = max(centersStrong4(:,1));
			y2 = max(centersStrong4(:,2));
			w = x2-x1;
			h = y2-y1;
			diagonal = 1.414 * sqrt(w^2+h^2);

			% check if red circles are in a square; aspect ratio of 1.3 and diagonal is 80% of width/ht
			if ( (w/h <=1.2 && w/h >=0.7) || (h/w <=1.2 && h/w >=0.7) ) && (diagonal >= 0.8*w/1.414 || diagonal >= 0.8*h/1.414 )
				w = x2-x1;
				h = y2-y1;
				centerBox_X = x1+w/2;
				centerBox_Y = y1+h/2;
				FLAG_redCircles = true;
			end
			%             FLAG_redCircles = true;
		else
			% draw blue circles
			hx = viscircles(centersStrong4, radiiStrong4 , 'Color','b');

			% If no Red Circles found (maybe light conditions etc; see if the circles are in a square
			% If circles are in a square; return the x1, y1, w and h values anyway
			% redundency in tracking; in case RED colour is hard to find

			circles = zeros(4,2);
			% get sorted x and y values)
			for a = 1:length(centersStrong4)
				circles(a,:) = centersStrong4(boundaryOrder(a),:);
			end

			Xcol = round(circles(:,1));
			Ycol = round(circles(:,2));
			% points
			% 1--2
			% 4--3
			l1 = sqrt( (Xcol(2)-Xcol(1))^2+(Ycol(2)-Ycol(1))^2);
			l2 = sqrt( (Xcol(3)-Xcol(2))^2+(Ycol(3)-Ycol(2))^2);
			l3 = sqrt( (Xcol(4)-Xcol(3))^2+(Ycol(4)-Ycol(3))^2);
			l4 = sqrt( (Xcol(1)-Xcol(4))^2+(Ycol(1)-Ycol(4))^2);

			d1 = sqrt( (Xcol(4)-Xcol(2))^2+(Ycol(4)-Ycol(2))^2);
			d2 = sqrt( (Xcol(3)-Xcol(1))^2+(Ycol(3)-Ycol(1))^2);
			perimeter = l1+l2+l3+l4;
			% Bretschneider's Formula
			area = 0.25 * sqrt( 4*d1^2*d2^2 - (l2^2+l4^2-l1^2-l3^2));

			circularity = (perimeter^2) / (4*pi*area) ;
			fprintf('\n Circularity = %1.1f', circularity)

			% check if the circles are in a square
			% circularity < 1.6 is square; <1.2 is circle
			if circularity > 1.2 && circularity <  1.3
%             if circularity == 1.3
%                 figure(); imshow(grayImg); hx = viscircles(centersStrong4, radiiStrong4 , 'Color','b');
%                 figure(3);
				% The 4 circles are in a square even though they arnt red (light issue maybe?)
				% increasing bounding box to cover the entire region with Min and Max of x and y cols
				x1 = min(Xcol);     % first point?
				y1 = min(Ycol);     % first point?
				% x1 = (max(Xcol) - min(Xcol))/2 + min(Xcol);
				% y1 = (max(Ycol) - min(Ycol))/2 + min(Ycol);
				x2 = Xcol(2);
				y2 = Ycol(2);

				w = max(Xcol);
				h = max(Ycol);

				% Sort accoriding to first col (x) .. that way we get closest and furthest to x axis
				sortedCircles = sortrows(circles);
				centerBox_X = abs( sortedCircles(4,1) - sortedCircles(1,1) )/2 + min(Xcol);
				centerBox_Y = abs( sortedCircles(4,2) - sortedCircles(1,1) )/2 + min(Ycol);
				% set flag that circles are found, and cart can move
				FLAG_redCircles = true;
			else
				% Nope circles are not in a square; hence its just some random circles beind detected
				x1 = 0;
				y1 = 0;
				x2 = 0;
				y2 = 0;
				w = 0;
				h = 0;
				centerBox_X = 0;
				centerBox_Y = 0;
				FLAG_redCircles = false;
			end

		end
end

%% calculateTurnDirDistandSpeed: Calculates which direction to turn based on the input from
%	ultrosonic sensor flags, direction and deviation from camera (KLT tracking)
function [ lastError, motorADir, motorBDir, speedA, speedD] = calculateTurnDirDistandSpeed (distanceFromCenter,lastError, ~)
	global Kp;                             	% REMEMBER we are using Kp*100 so this is really 10 !
	global Ki;                           		% REMEMBER we are using Ki*100 so this is really 1 !
	global Kd;                            	% REMEMBER we are using Kd*100 so this is really 100!
	global TargetSpeed;		% Target Motor Power
	global integral;                          	% the place where we will store our integral
	global derivative;                     	% the place where we will store the derivative

	global FLAG_moveLeft;
	global FLAG_moveRight;
    global PIDTarget;
    global PIDspeedA;
    global PIDspeedD;
    global axisLimits;
    global frameCount;

	% PID	0.60Kc	2KpdT ; Pc	KpPc ; (8dT) -  As per Ziegler-Nicholas method
	% taken from: http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
	% ----------------------------------------------------------
	% Parameter	Rise time	Overshoot	Settling time	Error at equilibrium
	% Kp	Decrease	Increase	Small change	Decrease
	% Ki	Decrease	Increase	Increase	Eliminate
	% Kd	Indefinite  small	Decrease	Decrease	None
	% ----------------------------------------------------------

	% Kp = const.
	% Ki = 2(Kp)(dT) / (Pc)
	% Kd = (Kp)(Pc) / ((8)(dT))

	%   Kp = 2.4;                             % Kp is multiplied - Must not exceed target power
	% 	Ki = 0.01;                           	% REMEMBER we are using Ki*100 so this is really 1 !
	% 	Kd = 0.1;

	% note: error is distanceFromCenter variable (in Pixels)
	integral = integral + distanceFromCenter;        				% integral = integral + error
% 	derivative = distanceFromCenter - lastError;     				% derivative = error - lastError
	TurnSpeed = round(Kp*distanceFromCenter+ Ki*integral + Kd*derivative);		% TurnPower = Kp*error + Ki*integral + Kd*derivative
	fprintf('\n TurnSpeed = %d', TurnSpeed);
	% Calculate motor target power. More power = more speed (hence turn)
	if FLAG_moveLeft == true
		% Increase A power
		speedA = TargetSpeed - TurnSpeed;                 					% the power level for the A motor
		speedD = TargetSpeed + TurnSpeed;                					% the power level for the D motor
	elseif FLAG_moveRight == true
		% Increase D power
		speedA = TargetSpeed + TurnSpeed;                 					% the power level for the A motor
		speedD = TargetSpeed - TurnSpeed;                					% the power level for the D motor
	end
	fprintf('\n speedA = %d',speedA);
	fprintf('\n speedD = %d \n\n',speedD);

	motorADir='forward';
	motorBDir='forward';
	% save lastError for next loop
	lastError = distanceFromCenter;                  				% save the current error so it can be the lastError next time around}

    % plot the PID
    figure(33);
    addpoints(PIDTarget,frameCount,double(TargetSpeed));
    if frameCount ~= 0 %&& k < length(x)
        axis([0, frameCount+1, -TargetSpeed, 2*TargetSpeed]) % reset x-axis limits
    end
    
    addpoints(PIDspeedA,frameCount,double(speedA));
    if frameCount ~= 0 %&& k < length(x)
        axis([0, frameCount+1, -TargetSpeed, 3*TargetSpeed]) % reset x-axis limits
    end
    
    addpoints(PIDspeedD,frameCount,double(speedD));
    if frameCount ~= 0 %&& k < length(x)
        axis([0, frameCount+1,  -TargetSpeed, 3*TargetSpeed]) % reset x-axis limits
    end
    drawnow limitrate
end

%% cartTurn: Turn to follow the QR code
function cartTurn (~, ~, speedA, speedD, FLAG_Stop)
	global leftMotor;
	global rightMotor;
	% check if coming from a stop or turn function call
	if FLAG_Stop == true
		SPEED = 20;				% Max 100 speed.
		resetRotation(leftMotor);                    % Reset motor rotation counter
		resetRotation(rightMotor);
		leftMotor.Speed = SPEED;                     % Set motor speed
		rightMotor.Speed = SPEED;
		start(leftMotor);                            % Start motor
		start(rightMotor);
	end
	leftMotor.Speed = speedA;
	rightMotor.Speed = speedD;
end

%% cartStop: Stop the cart asap
function cartStop()
	global legoEV3;
	global leftMotor;
	global rightMotor;

	global FLAG_moveStraight;
% 	global FLAG_Stop;
	stop(leftMotor);
	stop(rightMotor);

	% raise flags that cart has stopped
	FLAG_moveStraight = false;

	writeStatusLight(legoEV3,'red','solid');
end


%% Initialize all variables
function initAll ()

	global cam;
	global videoPlayer;
	global FLAG_runloop;
	global numPts;
	global frameCount;
	global arduinoDevice;
	global SR04_Left1;
	global SR04_Left2;
	global SR04_Right1;
	global SR04_Right2;
	global SR04_Center;
	global serialPort;
	global pointTracker;
	global videoFrame;
	global legoEV3;
	global FrontLegoUltrasonic;
	global leftMotor;
	global rightMotor;
	global lastR1;
	global lastR2;
	global oldtoc;
	global newtoc;
	global targetQRImg
    global PIDTarget;
    global PIDspeedA;
    global PIDspeedD;
    global axisLimits;
    global AnimatedLineFigure;
    global TargetSpeed;


		targetQRImg = imread('QR.png');
		%%%%%--------- Init for Ultrasonic Sensor --------%%%%%%
		% FIGURE FOR DISPLAYING ULTRASONIC SENSOR DETECTION RESULTS
		% IF DISTANCE > THRESH; FIGUREBG WILL BECOME RED
		H = figure(99); set(H,'Name','Ultrasonic Sensor Output','NumberTitle','off');
		% hard code poistion to screen bottom
		set( gcf, 'Units', 'pixels' );
		set( gcf, 'Position', [40 40 400 350] );

		hold on
		% plot the ultrasonic response
		plotVisually();

		%%%%%--------- Init for Serial Port over USB for Arduino Comm--------%%%%%%
		% 		serialPort = serial('COM4');
		% 		serialPort.BaudRate=9600;
		% 		fopen(serialPort);
		% 		serialPort.ReadAsyncMode = 'manual';

		%%%%%--------- Init for KLT and Camera --------%%%%%%
		cam = init_webcam();
		% Capture one frame to get its size.
		videoFrame = snapshot(cam);
		% Create a video player object for displaying video frames.
		videoPlayer  = vision.VideoPlayer('Position',[400+50 380 680, 500]);
		% run while window is open - else close and cleanup
		FLAG_runloop = true;
		numPts = 0;
		frameCount = 0;

		H = figure(3); set(H,'Name','Detected Circles in Frame','NumberTitle','off', 'Position', [30 480 400 350] );
		% 	set( gcf, 'Units', 'pixels' );
		 %  POSITION::    Left   Bottom     Width          Height
		imshow(videoFrame);
		% Initialize a Tracker to Track the Points   With the feature points identified, you can now use the
		% |vision.PointTracker| System object to track them. For each point in the  previous frame, the point tracker attempts to find the corresponding
		% point in the current frame.
		pointTracker = vision.PointTracker('MaxIterations', 30, 'BlockSize', [31,31], 'MaxBidirectionalError', 2);

		%%%%%---------  For Lego EV3 --------%%%%%%
		delete(legoEV3);                            % delete any existing connections
		legoEV3 = legoev3('USB');
		%         legoEV3 = legoev3('Bluetooth', 'COM5');
		FrontLegoUltrasonic = sonicSensor(legoEV3);
		leftMotor = motor(legoEV3 , 'C');              % Set up motor
		rightMotor = motor(legoEV3 , 'B');
		lastR1 = 0;
		lastR2 = 0;
		% Start a timer for PID control
		tic;
		oldtoc = 0;
		newtoc = 0;
        
        %%%%%---------  For PID plotting --------%%%%%%
        AnimatedLineFigure = figure(33); set(AnimatedLineFigure, 'Position', [450 40 400 350], 'Name','Target Speed vs Motor Speeds ', 'NumberTitle','off');
        PIDTarget = animatedline('Color','r'); 
        PIDspeedA = animatedline('Color','b');
        PIDspeedD = animatedline('Color','g');
        axis([0,50,-TargetSpeed,TargetSpeed])
        legend('TargetSpeed','speedA', 'speedD')
        axisLimits = axis; % get the current limits


end

%%
function [camera] = init_webcam()

	global cam;
	delete (cam);                   % delete any existing connections
	imaqreset;                      % reset any existing cameras
		cam = webcam('Logitech');
% 		cam = webcam('HD WebCam');
	cam.AvailableResolutions;
	cam.Resolution = '640x480';
	cam.Brightness = 150;
	camera = cam;
end

function plotVisually()
	figure(99);
	subplot(3,2,[1 2]);
		box off;
		set(gca,'xcolor',get(gcf,'color'));
		set(gca,'xtick',[],'ytick',[]);
		set(gca,'color','g');
	subplot(3,2,3);
		box off;
		set(gca,'xcolor',get(gcf,'color'));
		set(gca,'xtick',[],'ytick',[]);
		set(gca,'color','g');
	subplot(3,2,4);
		box off;
		set(gca,'xcolor',get(gcf,'color'));
		set(gca,'xtick',[],'ytick',[]);
		set(gca,'color','g');
	subplot(3,2,5);
		box off;
		set(gca,'xcolor',get(gcf,'color'));
		set(gca,'xtick',[],'ytick',[]);
		set(gca,'color','g');
	subplot(3,2,6);
		box off;
		set(gca,'xcolor',get(gcf,'color'));
		set(gca,'xtick',[],'ytick',[]);
		set(gca,'color','g');
end