% gleiche Perspektive für alle Kinematiken

clear
close all

robotList = {   'RobotFig_PlotNum1_RobGroup1_P6RRRRRR10V3G',...
                'RobotFig_PlotNum2_RobGroup2_P6RRRRRR10V6G',...
                'RobotFig_PlotNum3_RobGroup3_P6RRRRRR5V2G',...
                'RobotFig_PlotNum4_RobGroup4_P6RRRRRR6V2G',...
                'RobotFig_PlotNum5_RobGroup5_P6RRRRRR8V2G'};
view_props = {'View','CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle',...
                'OuterPosition','InnerPosition','DataAspectRatio','PlotBoxAspectRatioMode',...
                'Projection'};

%% get axes handles
fighdl = {};
axhdl = {};
for k = 1:length(robotList)
    openfig(robotList{k});
    figure(k)
    axhdl{k} = gca;
%     fighdl{k} = gcf;
end

% link properties
% hlink = linkprop([axhdl{1},axhdl{2},axhdl{3},axhdl{4},axhdl{5}],{'CameraPosition','CameraUpVector'});
% addprop(hlink,'CameraTarget')
% addprop(hlink,'CameraViewAngle')
% addprop(hlink,'View')

%% set properties
refIdxFig = 1; % alle anderen erhalten gleichen Attribute

for ii = 1:length(view_props)
    for k = 1:5
        set(axhdl{k},view_props{ii},get(axhdl{refIdxFig},view_props{ii}));
    end
end

%% export
for k = 1:length(robotList)
    fig = figure(k);
    export_fig([robotList{k}, '_r864.png'], '-r864');
end
