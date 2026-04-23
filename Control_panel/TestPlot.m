clear; clc; close all;

% 1. å½ˆå‡ºè¦–ç?è®“ä½¿?¨è€…é¸?‡è??œå??„è??™å¤¾
targetFolder = pwd;
if targetFolder == 0
    error('?ä?å·²å?æ¶ˆï??ªé¸?‡ä»»ä½•è??™å¤¾??);
end

% ?œå?è©²è??™å¤¾ (?…å«?€?‰å?è³‡æ?å¤? ä¸‹æ??‰ç? return_trajectory æª”æ?
filePattern = fullfile(targetFolder, '**', 'return_trajectory_*.txt');
files = dir(filePattern);

if isempty(files)
    error('?¨è??™å¤¾ %s ?Šå…¶å­è??™å¤¾ä¸­æ‰¾ä¸åˆ°ä»»ä? return_trajectory çµå°¾??txt æª”æ?ï¼?, targetFolder);
end

% ?ªå??¾å‡º?€?°ä¿®?¹ç?æª”æ?ï¼Œä»¥ä¾¿æ??–æ??°ç? timestamp
[~, latest_idx] = max([files.datenum]);
newestFile = files(latest_idx).name;

if isfield(files, 'folder')
    baseFolder = files(latest_idx).folder;
else
    baseFolder = targetFolder;
end

% å¾æ??ä¸­?å??‚é??³è? (ä¾‹å? 20260420_112856)
timestampStr = regexp(newestFile, '\d{8}_\d{6}', 'match', 'once');
if isempty(timestampStr)
    error('?¡æ?å¾æ??è¾¨è­˜æ??“æˆ³è¨?(%s)', newestFile);
end

% ?é??Œä??‹æ??“æˆ³è¨˜ï??Œæ??¾å? Pose (??Yaw/Pitch/XYZ) ??Motor (?‰é?ç¯€è§’åº¦) æª”æ?
poseFile = fullfile(baseFolder, ['return_trajectory_pose_' timestampStr '.txt']);
motorFile = fullfile(baseFolder, ['return_trajectory_motor_' timestampStr '.txt']);

fprintf('--- ?å??¾åˆ°ä¸¦æ??™è??–å…©ä»½è?è·¡è???---\n');
fprintf('Pose æª”æ?: %s\n', poseFile);
fprintf('Motor æª”æ?: %s\n', motorFile);

% ==================== è®€?–ä¸¦?•ç? Pose è³‡æ? ====================
if ~exist(poseFile, 'file')
    error('?¾ä??°å??‰ç? Pose æª”æ?: %s', poseFile);
end
% ä½¿ç”¨ readmatrix ?–ä»£?Ÿæœ¬??regular expressionï¼Œå¯ä»¥é¿?å??ºæ­£è¦è¡¨?”å?è§??å°æ•¸é»ç??°å¸¸å°è‡´?€å¾Œä?ç­†ç¼ºå¤?
poseData = readmatrix(poseFile); 
% æ¿¾é™¤??NaN ä¹Ÿå°±?¯ä?å®Œæ•´?„è??™å? (è§?±º?å‚³è³‡æ?ä¸å??´å??´ç?ç¹ªå??¯èª¤)
poseData = poseData(~any(isnan(poseData), 2), :);

yaw   = poseData(:, 1); 
pitch = poseData(:, 2); 
z_abs = poseData(:, 3); 
y_abs = poseData(:, 4); 
x_abs = poseData(:, 5);

% ==================== è®€?–ä¸¦?•ç? Motor è³‡æ? ====================
if ~exist(motorFile, 'file')
    error('?¾ä??°å??‰ç? Motor æª”æ?: %s', motorFile);
end
motorData = readmatrix(motorFile);
motorData = motorData(~any(isnan(motorData), 2), :);

theta1 = motorData(:, 1);
theta2 = motorData(:, 2);
theta3 = motorData(:, 3);
theta4 = motorData(:, 4);
theta5 = motorData(:, 5);

% å°é??‚é?è»?(?–æ?å¼·ç¡¬?ªæ–·ï¼Œä??™å??ªç¨ç«‹ç?å®Œæ•´?·åº¦)
t_pose = (1:length(yaw))';
t_motor = (1:length(theta1))';

% ==================== ç¹ªå??€ ====================
figure('Color', 'w', 'Name', 'Trajectory & Joint Analysis', 'Position', [100, 100, 1200, 800]);

% (1) ?«ç«¯é»?3D ä½ç½®??(å·¦å´? æ?ä¸Šä??©å€?span)
subplot(2, 2, [1, 3]);
plot3(x_abs, y_abs, z_abs, 'r', 'LineWidth', 2); hold on;
plot3(x_abs(1), y_abs(1), z_abs(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % èµ·é?
plot3(x_abs(end), y_abs(end), z_abs(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % çµ‚é?
grid on; 

view_range = max(max(x_abs)-min(x_abs), 2) / 2 + 1; 
cx = mean(x_abs); cy = mean(y_abs); cz = mean(z_abs);
axis([cx-view_range, cx+view_range, cy-view_range, cy+view_range, cz-view_range, cz+view_range]);

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('\bf 1. ?«ç«¯é»ä?ç½?(End-Effector Absolute Path)');
view(3);

% (2) å§¿æ?è§?(Yaw / Pitch) è®Šå???(?³ä?)
subplot(2, 2, 2);
plot(t_pose, yaw, 'b', 'LineWidth', 1.5); hold on;
plot(t_pose, pitch, 'm', 'LineWidth', 1.5);
grid on;
legend('Yaw (deg)', 'Pitch (deg)', 'Location', 'best');
xlabel('Sample Index (t)'); ylabel('Angle (deg)');
title('\bf 2. Yaw \ Pitch è§’åº¦è®Šå?');

% (3) ?„é?ç¯€è§’åº¦è®Šå???(?³ä?)
subplot(2, 2, 4);
plot(t_motor, theta1, 'LineWidth', 1.5); hold on;
plot(t_motor, theta2, 'LineWidth', 1.5);
plot(t_motor, theta3, 'LineWidth', 1.5);
plot(t_motor, theta4, 'LineWidth', 1.5);
plot(t_motor, theta5, 'LineWidth', 1.5);
grid on;
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', 'Location', 'best');
xlabel('Sample Index (t)'); ylabel('Joint Offset Angle');
title('\bf 3. ?„é?ç¯€è§’åº¦ (\theta) ?„è??–å?');
