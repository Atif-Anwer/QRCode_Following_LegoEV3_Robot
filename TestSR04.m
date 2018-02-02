
    global cam;
    global videoPlayer;
    global runLoop;
    global numPts;
    global frameCount;
    global arduinoDevice;
    global moveForward;
    global SR04_Left1;
    global SR04_Left2;
    global SR04_Right1;
    global SR04_Right2;
    global SR04_Center;
    global rightTurnAllowed;
    global leftTurnAllowed;
    global SR04Value;
    global serialPort;
    
while(1)
    
    fprintf(serialPort,'R');
    
    % Data will get values sperated by spaces, ending with a EOL char
    data = fscanf(serialPort);
    % split the string into values, convert to number array
    
    % ------->  Get 1st value, split the remainder, get val from remainder
    %           untill all 5 sensor values are read from the string
    [str,rem] = strtok(data);
    SR04Value(1) = real(str2double(str));
    for i = 2:5
        SR04Value(i) = real(str2double(strtok(rem)));
    end
        
    % display on the UI
    if SR04Value(3) <= 30
        % if object in front
        moveForward = false;
        figure(99);
        subplot(3,2,[1 2]);
        set(gca,'color','r');
        break; %#ok<BRKCONT>
    elseif SR04Value(1) <= 60
        % if object on left side
        leftTurnAllowed = false;
        figure(99);
        subplot(3,2,5);
        set(gca,'color','r');
    elseif SR04Value(2) <= 60
        % if object on left side
        leftTurnAllowed = false;
        figure(99);
        subplot(3,2,3);
        set(gca,'color','r');
    elseif SR04Value(4) <= 60
        % if object on right side
        rightTurnAllowed = false;
        figure(99);
        subplot(3,2,4);
        set(gca,'color','r');
    elseif SR04Value(5) <= 60
        % if object on right side
        rightTurnAllowed = false;
        figure(99);
        subplot(3,2,6);
        set(gca,'color','r');
    else
        % all clear
        moveForward = true;
        leftTurnAllowed = true;
        rightTurnAllowed = true;
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
    drawnow limitrate;
end
    
    