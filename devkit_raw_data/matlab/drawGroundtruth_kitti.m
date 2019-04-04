%% display groundtruth of KITTI poses
% include sequence from 00-10.txt

% read from poses
filename = '/dataset/KITTI/poses/07.txt';
fid = fopen(filename);
fseek(fid, 0, 'bof');
lastposition = ftell(fid);
disp(['start position:',num2str(lastposition)]);

groundtruth = [];

while fgetl(fid) ~= -1, % end of line check

    fseek(fid, lastposition, 'bof');
    line = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f\n',1);
    line = [line{:}];
    transform = vec2mat(line,4);

    groundtruth = [groundtruth; [transform(1,4), transform(3,4)]];
    lastposition = ftell(fid);
    disp(['lastposition:',num2str(lastposition)]);

end

filename2 = '/dataset/KITTI/poses/07_kitti_trajectory.txt';
fid2 = fopen(filename2);
fseek(fid2, 0, 'bof');
lastposition2 = ftell(fid2);
disp(['start position:',num2str(lastposition2)]);

trajectory = [];

while fgetl(fid2) ~= -1, % end of line check

    fseek(fid2, lastposition2, 'bof');
    line2 = textscan(fid2,'%f %f %f %f %f %f %f %f %f %f %f %f\n',1);
    line2 = [line2{:}];
    transform2 = vec2mat(line2,4);
    
    trajectory = [trajectory; [-transform2(1,1), transform2(1,3)]];
    lastposition2 = ftell(fid2);
    disp(['lastposition:',num2str(lastposition2)]);

end

[row1,~] = size(groundtruth);
[row2,~] = size(trajectory);
min_rows = min(row1,row2);
disp(['min_rows:',num2str(min_rows)]);

x0 = trajectory(1,1);
y0 = trajectory(1,2);
b  = 0.55906584;
trajectory2 = trajectory;

for j = 2:row2
    x = trajectory(j,1);
    y = trajectory(j,2);
    a = pi - atan2(y-y0,x-x0);
    d = sqrt((y-y0)^2 + (x-x0)^2);
    trajectory2(j,1) = x0 - d*cos(a-b);
    trajectory2(j,2) = y0 + d*sin(a-b);
end

    
% display ground truth and trajectory
scatter(groundtruth(:,1),groundtruth(:,2));
plot(groundtruth(:,1),groundtruth(:,2),'k-.',trajectory2(:,1),trajectory2(:,2),'b','LineWidth',3);
legend('groundtruth','trajectory');
xlabel('X Position [m]');
ylabel('Y Position [m]');


% draw the difference between groundtruth and trajectory

delta_p = abs(groundtruth(1:min_rows,1:2) - trajectory(1:min_rows,1:2));
delta_p(:,3) = sqrt(delta_p(:,1).^2 + delta_p(:,2).^2);

path_length(1,1) = 0;
for r = 2:min_rows
    delta_length = (groundtruth(r,1)-groundtruth(r-1,1)).^2 + (groundtruth(r,2)-groundtruth(r-1,2)).^2;
    path_length(r,1) = path_length(r-1,1) + sqrt(delta_length);
end

figure;
plot(delta_p,'LineWidth',5);
grid on;
xlabel('Frame Number');
ylabel('Translation Error [m]');
legend('△x','△y','△d');

% draw error
for i = 2:min_rows
    trans_error(i,1) = 100*delta_p(i,3)/path_length(min_rows,1);
end
trans_error(1,1) = trans_error(2,1);

figure;
plot(path_length,trans_error,'LineWidth',5);
grid on;
xlabel('Path Length [m]');
ylabel('Translation Error [%]');

fclose(fid);
fclose(fid2);