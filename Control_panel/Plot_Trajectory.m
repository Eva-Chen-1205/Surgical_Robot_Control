clear; clc; close all;

% 1. 彈出視窗讓使用者選擇要搜尋的資料夾
targetFolder = uigetdir(pwd, '請選擇預設包含 return_trajectory_*.txt 檔案的資料夾');
if targetFolder == 0
    error('操作已取消：未選擇任何資料夾。');
end

% 搜尋該資料夾 (包含所有子資料夾) 下所有的 return_trajectory 檔案
filePattern = fullfile(targetFolder, '**', 'return_trajectory_*.txt');
files = dir(filePattern);

if isempty(files)
    error('在資料夾 %s 及其子資料夾中找不到任何 return_trajectory 結尾的 txt 檔案！', targetFolder);
end

% 自動找出最新修改的檔案，以便抓取最新的 timestamp
[~, latest_idx] = max([files.datenum]);
newestFile = files(latest_idx).name;

if isfield(files, 'folder')
    baseFolder = files(latest_idx).folder;
else
    baseFolder = targetFolder;
end

% 從檔名中提取時間戳記 (例如 20260420_112856)
timestampStr = regexp(newestFile, '\d{8}_\d{6}', 'match', 'once');
if isempty(timestampStr)
    error('無法從檔名辨識時間戳記 (%s)', newestFile);
end

% 透過同一個時間戳記，同時找回 Pose (有 Yaw/Pitch/XYZ) 與 Motor (有關節角度) 檔案
poseFile = fullfile(baseFolder, ['return_trajectory_pose_' timestampStr '.txt']);
motorFile = fullfile(baseFolder, ['return_trajectory_motor_' timestampStr '.txt']);

fprintf('--- 成功找到並準備讀取兩份軌跡資料 ---\n');
fprintf('Pose 檔案: %s\n', poseFile);
fprintf('Motor 檔案: %s\n', motorFile);

% ==================== 讀取並處理 Pose 資料 ====================
if ~exist(poseFile, 'file')
    error('找不到對應的 Pose 檔案: %s', poseFile);
end
% 使用 readmatrix 取代原本的 regular expression，可以避免因為正規表達式解析小數點的異常導致最後一筆缺失
poseData = readmatrix(poseFile); 
% 濾除有 NaN 也就是不完整的資料列 (解決回傳資料不完整導致的繪圖錯誤)
poseData = poseData(~any(isnan(poseData), 2), :);

yaw   = poseData(:, 1); 
pitch = poseData(:, 2); 
z_abs = poseData(:, 3); 
y_abs = poseData(:, 4); 
x_abs = poseData(:, 5);

% ==================== 讀取並處理 Motor 資料 ====================
if ~exist(motorFile, 'file')
    error('找不到對應的 Motor 檔案: %s', motorFile);
end
motorData = readmatrix(motorFile);
motorData = motorData(~any(isnan(motorData), 2), :);

theta1 = motorData(:, 1);
theta2 = motorData(:, 2);
theta3 = motorData(:, 3);
theta4 = motorData(:, 4);
theta5 = motorData(:, 5);

% 對齊時間軸
t_pose = (1:length(yaw))';
t_motor = (1:length(theta1))';

% ==================== 繪圖區 ====================
figure('Color', 'w', 'Name', 'Trajectory & Joint Analysis', 'Position', [100, 100, 1200, 800]);

% (1) 末端點 3D 位置圖 (左側占據上下兩個 span)
subplot(2, 2, [1, 3]);
plot3(x_abs, y_abs, z_abs, 'r', 'LineWidth', 2); hold on;
plot3(x_abs(1), y_abs(1), z_abs(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8); % 起點
plot3(x_abs(end), y_abs(end), z_abs(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % 終點
grid on; 

view_range = max(max(x_abs)-min(x_abs), 2) / 2 + 1; 
cx = mean(x_abs); cy = mean(y_abs); cz = mean(z_abs);
axis([cx-view_range, cx+view_range, cy-view_range, cy+view_range, cz-view_range, cz+view_range]);

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('\bf 1. End-Effector Absolute Path');
view(3);

% (2) 姿態角 (Yaw / Pitch) 變化圖 (右上)
subplot(2, 2, 2);
plot(t_pose, yaw, 'b', 'LineWidth', 1.5); hold on;
plot(t_pose, pitch, 'm', 'LineWidth', 1.5);
grid on;
legend('Yaw (deg)', 'Pitch (deg)', 'Location', 'best');
xlabel('Sample Index (t)'); ylabel('Angle (deg)');
title('2. Yaw \ Pitch angle');

% (3) 各關節角度變化圖 (右下)
subplot(2, 2, 4);
plot(t_motor, theta1, 'LineWidth', 1.5); hold on;
plot(t_motor, theta2, 'LineWidth', 1.5);
plot(t_motor, theta3, 'LineWidth', 1.5);
plot(t_motor, theta4, 'LineWidth', 1.5);
plot(t_motor, theta5, 'LineWidth', 1.5);
grid on;
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', 'Location', 'best');
xlabel('Sample Index (t)'); ylabel('Joint Offset Angle');
title('3. changes in joint angles (θ)');