%% Initialization 
%Initilizes XBee receiver
xbee = serial('COM8');
set(xbee,'BaudRate',115200)
fopen(xbee);

%prepares variables
data = zeros(1000,2);
movingAverage = zeros(1000,2);
i = 1;
invalidDataCount = 0;

%Creates GUI Window
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
              'String', 'Break', ...
              'Callback', 'delete(gcbf)');

%initilazation of variables for live plotting
h = animatedline;
axisMin = 0;
axisMax= 1e-10;
axis([axisMin axisMax -90 90])


%% Reading data from XBee
try % if there is ever an error, serial port is closed. If this doesnot happen,
    % then the serial port will remain open and unusable.
    while(ishandle(H)) %while escape button is not pressed
        
        incomingData = fgetl(xbee); %reads data
        
        %validates form of incoming data and calculates it
        if isempty(incomingData) %checks if a packet was truly received
            fclose(xbee);
            disp("Failed: fetched null packet");
            break
        elseif  contains(incomingData,"?") ||...
                contains(incomingData,"°") ||...
                contains(incomingData,"µ") ||...
                contains(incomingData,"¸") 
                %checks for incorrect transmission markers
            invalidDataCount = invalidDataCount +1;
        else
            %formats data into two usable strings
            parsedData = split(incomingData, ",");
            
            data(i,:) = str2double(parsedData);
            data(i,2) = (data(i,2)-data(1,2))/1000;%zeros and converts time
            
            if i>5 %makes sure there is adequate data for moving average
                movingAverage(i,1) = mean(data(i-5:i,1));
                movingAverage(i,2) = data(i,2);
                
                %graphs
                addpoints(h,movingAverage(i,2),movingAverage(i,1));
                if i>1000 %allows the axis / graph to "Move" with the incoming data
                    axisMin = movingAverage(i-999,2);
                end
                axisMax = movingAverage(i,2)+1;
                axis([axisMin axisMax -90 90]);
                drawnow limitrate
                
            end
            i=i+1;
        end
    end
    fclose(xbee); %Closes serial port. MUST BE DONE
catch
    fclose(xbee);
    disp("Error occured, serial port closed")
end

%% formats and prints the data
data = data(any(data,2),:); %removes any empty rows
data = data(5:end,:); %removes the first few outlier points
movingAverage = movingAverage(any(movingAverage,2),:);
movingAverage = movingAverage(5:end,:);%removes the first few outlier point
dt = diff(data(:,2));%converts time stamps to delta t
dt=dt(5:end); %removes the first few outlier point

disp(" ")
disp("Percentage of data read corretly= " + (i/(invalidDataCount+i))*100 +"%");
disp("Average dt of correct data= " + mean(dt) + " s");
disp("Average correct refresh rate= " + 1/(mean(dt)) + " Hz");

figure;
subplot(2,1,1);
plot(data(:,2),data(:,1));
title("Y angle vs time");
subplot(2,1,2);
plot(dt);
title("delta t");

figure
plot(movingAverage(:,2),movingAverage(:,1))
title("Moving Average Approximation of Angle");
xlabel("Time (s)")
ylabel("Roll Angle (°)")




