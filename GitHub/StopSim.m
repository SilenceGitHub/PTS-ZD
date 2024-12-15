function StopSim()
global data param robot

% save(['Data/Data_',param.acf,'_step_1e-3'], 'data', 'param', 'robot');
% load data/InitData1.mat
[all_themes, all_colors] = GetColors();
Error = data.desiredPath - data.actualPath;
t = 0:param.step:param.taskDuration;
samplingNumber = round(param.taskDuration/param.step) + 1;

interval = 10;
figure
plot3(data.desiredPath(1,1:interval:end),data.desiredPath(2,1:interval:end),data.desiredPath(3,1:interval:end),'r-');
hold on;
plot3(data.actualPath(1,1:interval:end),data.actualPath(2,1:interval:end),data.actualPath(3,1:interval:end),'g--');
axis equal


figure;
p1 = plot(t(1:interval:end),Error(1,1:interval:end)','-.', 'linewidth', 3);
hold on;grid on;
p2 = plot(t(1:interval:end),Error(2,1:interval:end)','--', 'linewidth', 3);
p3 = plot(t(1:interval:end),Error(3,1:interval:end)',':', 'linewidth', 3);
start_n = round(param.t_p/param.step)+1;
total_n = round((param.taskDuration - param.t_p)/param.step)+1;
RMSE = round(sqrt(sum(Error(1,start_n:end).^2 + Error(2,start_n:end).^2 + Error(3,start_n:end).^2)/total_n), 8)
annotation = strcat('RMSE=',mat2str(RMSE),' mm');
% text(5, 0.8, annotation, 'fontsize', 20,'FontName', 'times new Roman');
legend([p1, p2, p3] ,'$e_{p,x}$','$e_{p,y}$','$e_{p,z}$','Location','best', 'FontName', 'times new Roman','fontsize',30,'interpreter','latex','NumColumns',3);
hold off;
set(gca,'FontSize',30, 'FontName', 'times new Roman');
xlabel('$t$ (s)', 'FontName', 'times new Roman','fontsize',30,'interpreter','latex');
ylabel('Error (m)', 'FontName', 'times new Roman','fontsize',30);



q1 = data.inputAngle(1,:); q2 = data.inputAngle(2,:); q3 = data.inputAngle(3,:); q4 = data.inputAngle(4,:); q5 = data.inputAngle(5,:); q6 = data.inputAngle(6,:);
figure;
plot(t, q1, '-', 'linewidth', 3);%grid on;
hold on; grid on;
plot(t, q2, '--', 'linewidth', 3);
plot(t, q3, ':', 'linewidth', 3);
plot(t, q4, '-', 'linewidth', 3);
plot(t, q5, '--', 'linewidth', 3);
plot(t, q6, ':', 'linewidth', 3);
hold off;
set(gca,'FontSize',20);
xlabel('t (s)', 'FontName', 'times new Roman','fontsize',20);
ylabel('Actuation input', 'FontName', 'times new Roman','fontsize',20);
legend('$\varphi_l$', '$\varphi_r$', '$\theta_1$','$\theta_2$', '$\theta_3$', '$\theta_4$', 'FontName', 'times new Roman','fontsize',20,'interpreter','latex');

interval1 = 10;


interval = 200;
fontsize = 16;
figure;
for i = 1:4
subplot(2,2,i);
step = (i-1)*2000+1;
plot3(data.desiredPath(1,1:interval:end),data.desiredPath(2,1:interval:end),data.desiredPath(3,1:interval:end),'-', 'color', all_colors(25, :),'linewidth',3);
hold on;
plot3(data.actualPath(1,1:interval:step),data.actualPath(2,1:interval:step),data.actualPath(3,1:interval:step),'--x', 'color', all_colors(26, :),'linewidth',3);
for k = step
    linkPos = robot.LinkPosition(robot, data.inputAngle(:,k));
     grid on;
    plot3(linkPos(1,1:2),linkPos(2,1:2),linkPos(3,1:2), 'color', all_colors(5, :),'linewidth',3);
    plot3(linkPos(1,2:3),linkPos(2,2:3),linkPos(3,2:3), 'color', all_colors(1, :),'linewidth',3);
    plot3(linkPos(1,3:4),linkPos(2,3:4),linkPos(3,3:4), 'color', all_colors(2, :),'linewidth',3);
    plot3(linkPos(1,4:5),linkPos(2,4:5),linkPos(3,4:5), 'color', all_colors(3, :),'linewidth',3);
    plot3([linkPos(1,1),linkPos(1,6)],[linkPos(2,1),linkPos(2,6)],[linkPos(3,1),linkPos(3,6)],'k','linewidth',2);
    plot3([linkPos(1,1),linkPos(1,7)],[linkPos(2,1),linkPos(2,7)],[linkPos(3,1),linkPos(3,7)],'k','linewidth',2);
    plot3([linkPos(1,6),linkPos(1,7)],[linkPos(2,6),linkPos(2,7)],[linkPos(3,6),linkPos(3,7)],'k','linewidth',2);

%     rectangle('Position',[midpoint(1)-b,midpoint(2)-b,2*b,2*b],'Curvature',[1,1]);
    scatter3(linkPos(1,6:7),linkPos(2,6:7),linkPos(3,6:7),300,'filled','MarkerEdgeColor','g', 'MarkerFaceColor','k','linewidth',1);
end
hold off;
axis equal;
box on;
axis([-1 0 -0.4 0.7 0 0.45]);
view(3);
set(gca,'FontSize',fontsize,'fontname','times new Roman');
xlabel('X (m)','fontsize',fontsize,'fontname','times new Roman');
ylabel('Y (m)','fontsize',fontsize,'fontname','times new Roman');
zlabel('Z (m)','fontsize',fontsize,'fontname','times new Roman');
legend('Reference trajectory','Actual trajectory','Location','best', 'FontName', 'times new Roman','fontsize',fontsize,'NumColumns',2);
end
set(gcf, 'color','white');

end




function [all_themes, all_colors] = GetColors()

c_string{1} = {'#FD6D5A', '#FEB40B', '#6DC354', '#994487', '#518CD8', '#443295'};

c_string{2} = {'#264653', '#2A9D8F', '#E9C46A', '#F4A261', '#E76F51', '#253777'};

c_string{3} = {'#C1C976', '#C8A9A1', '#FEC2E4', '#77CCE0', '#FFD372', '#F88078'};

c_string{4} = {'#104FFF', '#2FD151', '#64C7B8', '#FF1038', '#45CAFF', '#B913FF'};

c_string{5} = {'#4C87D6', '#F38562', '#F2B825', '#D4C114', '#88B421', '#199FE0'};

c_string{6} = {'#037CD2', '#00AAAA', '#927FD3', '#E54E5D', '#EAA700', '#F57F4B'};

c_string{7} = {'#64B6EA', '#FB8857', '#A788EB', '#80D172', '#FC7A77', '#61D4D5'};

c_string{8} = {'#F1787D', '#F8D889', '#69CDE0', '#5EB7F1', '#EDA462', '#F6C4E6'};

c_string{9} = {'#8C8FD5', '#C0E5BC', '#8C8FD5', '#BDF4FC', '#C3BCE6', '#F48FB1'};

c_string{10} = {'#B04A7A', '#171433', '#B6342E', '#DBB9DB', '#FAB4AC', '#EFB9C1'};

c_string{11} = {'#E74745', '#FB7857', '#FBCD60', '#FEFB66', '#1AC0C6', '#FB7857'};

c_string{12} = {'#361D32', '#543C52', '#F65A53', '#EED2CB', '#DBD873', '#F1E8E8'};

c_string{13} = {'#454D66', '#319975', '#58B368', '#DBD873', '#FAC46C', '#F1ECB7'};

c_string{14} = {'#112E92', '#112E92', '#48D6D2', '#81EAE6', '#F8F6BA', '#E3F5F6'};

c_string{15} = {'#134036', '#103232', '#34C0B8', '#7A27FF', '#FFCA7B', '#F8A427'};

c_string{16} = {'#FF0000', '#F65A53', '#34C0B8', '#7A27FF', '#FF98A4', '#D7BCE7'};

c_string{17} = {'#F84D4D', '#FF6B42', '#5BA3EB', '#06BB9A', '#8E7EF0', '#F4B919'};

c_string{18} = {'#406196', '#F4B414', '#77649B', '#385D77', '#576270', '#778495'};

c_string{19} = {'#4C6CFF', '#18C1FF', '#3DEF2D', '#9818BC', '#CB1E86', '#FC564B'};

c_string{20} = {'#0E9DFF', '#FF0000', '#800080', '#FFA500', '#ECE70B', '#979797'};

c_string{21} = {'#313BD0', '#9A22F8', '#00F2F2', '#00B2FC', '#5EAFFA', '#81E0D7'};

c_string{22} = {'#55EFC4', '#81ECEC', '#74B9FF', '#A29BFE', '#7F8C8D', '#C9C9C9'};

c_string{23} = {'#FAD390', '#F8C291', '#3742FA', '#70A1FF', '#82CCDD', '#B8E994'};

c_string{24} = {'#F8E0F1', '#DEC8CC', '#B87D9C', '#82447F', '#0B174E', '#C9C9C9'};

c_string{25} = {'#0063C3', '#408AD2', '#3396CF', '#A0D284', '#F5CD39', '#BEBEBE'};
c_string{26} = {'#4316F3', '#8769F7', '#FA807C', '#EFC09D', '#A5A5A5', '#C9C9C9'};

c_string{27} = {'#104382', '#0A8CB2', '#448CE1', '#1BB5E1', '#FF6155', '#C9C9C9'};

c_string{28} = {'#14BF96', '#1865F2', '#FFB100', '#073587', '#5F9ABC', '#BCBCBC'};

c_string{29} = {'#16B99A', '#1F7CC1', '#1686F8', '#3EC5E0', '#93D06A', '#BABABA'};

c_string{30} = {'#014AE0', '#1587FD', '#2DD3FB', '#FA6766', '#F8B613', '#BD55F9'};

c_string{31} = {'#1987D7', '#5B73CD', '#1FBF79', '#3BB9C1', '#F86B0D', '#FAA900'};

c_string{32} = {'#4071EC', '#FA3279', '#1096FA', '#35B9E4', '#2DCC97', '#A0B2BA'};

c_string{33} = {'#007EB4', '#00A7FC', '#FFC85C', '#ED326C', '#A84269', '#BC4715'};

c_string{34} = {'#F79F1F', '#A3CB38', '#1289A7', '#D980FA', '#FDA7DF', '#B53471'};

c_string{35} = {'#FA983A', '#EB2F06', '#1E3799', '#3C6382', '#38ADA9', '#C9C9C9'};

c_string{36} = {'#2ED573', '#1E90FF', '#3742FA', '#70A1FF', '#2F3542', '#C9C9C9'};
% 
c_string{37} = {'#FC5C65', '#FD9644', '#FED330', '#26DE81', '#079992', '#2BCBBA'};
% 
c_string{38} = {'#F3A683', '#F7D794', '#778BEB', '#70A1FF', '#E77F67', '#CF6A87'};
% 
c_string{39} = {'#F6B93B', '#E55039', '#4A69BD', '#60A3BC', '#78E08F', '#BBEA99'};
% 
c_string{40} = {'#00ADD7', '#00668A', '#F1C900', '#FFF1CE', '#E01706', '#C9C9C9'};
% 
c_string{41} = {'#D3DE9E', '#004965', '#17C0EB', '#FF7F78', '#3D3D3D', '#FFF200'};
% 
c_string{42} = {'#CD84F1', '#FFCCCC', '#FF4D4D', '#FFAF40', '#2F3542', '#FFFA65'};
% 
c_string{43} = {'#F53B57', '#3C40C6', '#3C40C6', '#00D8D6', '#05C46B', '#C9C9C9'};
%  
c_string{44} = {'#50514F', '#F45E58', '#FFE15B', '#1D7AA2', '#6DC1B3', '#A5B1C2'};
% 
c_string{45} = {'#FFC312', '#12CBC4', '#C4E538', '#1289A7', '#FDA7DF', '#ED4C67'};
% 
c_string{46} = {'#FFC000', '#EA3C6E', '#1B73A7', '#2C946E', '#A5B1C2', '#303952'};
% 
c_string{47} = {'#A589C6', '#FD91A0', '#F2E9DA', '#DFE384', '#39BFCB', '#A6E3E8'};
% 
c_string{48} = {'#CC99FF', '#FFAFD7', '#9BCDFF', '#FFD0A1', '#99FFCC', '#CCFF9A'};
% 
c_string{49} = {'#3F48CC', '#B83DBA', '#FF7F27', '#0ED145', '#EC1C24', '#A76E4E'};
% 
c_string{50} = {'#385261', '#7298AB', '#4F9A73', '#74878B', '#86AEA6', '#D1F0F3'};
% 
c_string{51} = {'#ED5736', '#C61D34', '#F30D00', '#DD5A6C', '#F00057', '#FE0096'};
% 
c_string{52} = {'#4B59C2', '#5A77BB', '#4A94C5', '#3B2E7E', '#013370', '#1A2946'};
% 
c_string{53} = {'#08DE9E', '#01E400', '#21A576', '#BDDD20', '#9DBC20', '#67945B'};
% 
c_string{54} = {'#7F1EAC', '#815377', '#57004F', '#4C211B', '#CBA4E5', '#A5A9D6'};
% 
c_string{55} = {'#2D445F', '#3F4D50', '#494263', '#2279B6', '#7B9067', '#B56B62'};
% 
c_string{56} = {'#101420', '#4C000A', '#1A5599', '#8E2961', '#407D53', '#8E2961'};
% 
c_string{57} = {'#0095B6', '#4FA485', '#81D8D0', '#E2AF42', '#B8CE8E', '#9AB4CD'};
% 
c_string{58} = {'#005EAD', '#AF6DE5', '#719FFB', '#1CAC99', '#FE9499', '#4A8FDE'};
% 
c_string{58} = {'#4DE890', '#2178B8', '#77A2E8', '#F86067', '#26C4B8', '#0094C5'};
% 
c_string{58} = {'#5AE7E4', '#2E9F79', '#3638AE', '#FF7F00', '#FA9B97', '#30A02D'};
% 
c_string{58} = {'#B0E188', '#2077B5', '#05B9C7', '#A8CBE4', '#F5FFB3', '#BEECAF'};
% 
c_string{59} = {'#FFA1C4', '#8770E0', '#01AFEE', '#4574C6', '#FDC100', '#BAD0C4'};
% 
c_string{60} = {'#4AB9EE', '#FF178D', '#FF178D', '#FFD600', '#00B1A1', '#97D601'};
% 
% 
m_count = length(c_string);
c_count = sum(cellfun(@length, c_string));
all_colors = nan(c_count, 3);
all_themes = cell(m_count, 1);
idx = 1;
for i = 1 : length(c_string)
    color = nan(length(c_string{i}), 3);
    for j = 1 : length(c_string{i})
        %
        rgb = Hex2RGB(c_string{i}{j});
        all_colors(idx, :) = RGB2MatlabColor(rgb);
        color(j, :) = all_colors(idx, :);
        idx = idx + 1;
    end
    all_themes{i} = color;
end
end

function c = Hex2RGB(str)
    clist = '0123456789ABCDEF';
    nums = zeros(1, 6);
    for i = 1:6
        nums(i) = find(str(i + 1) == clist);
    end
    c = zeros(1, 3);
    for i = 1:3
        c(i) = 16 * (nums(2*i-1) - 1) + (nums(2*i) - 1);
    end
end

function c = RGB2MatlabColor(rgb)
    c = round(100 * rgb/255) / 100; 
end