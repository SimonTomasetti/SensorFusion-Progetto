function BNO055_Data_Plotter()
    % Versione semplificata garantita per i grafici 2D
    try
        % Caricamento dati con fallback al CSV
        if exist('imu_data.mat', 'file')
            load('imu_data.mat');
        else
            data = readtable('imu_data.csv');
            timestamps = data.Timestamp;
            eulerAngles = [data.Roll, data.Pitch, data.Yaw];
        end
        
        % Creazione figura per i grafici 2D
        figure('Name', 'BNO055 Angles - SOLO 2D', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);
        
        % Plot Roll (semplice e garantito)
        subplot(3,1,1);
        plot(timestamps, eulerAngles(:,1), 'r-');
        title('Roll Angle');
        xlabel('Time (s)');
        ylabel('Degrees');
        grid on;
        ylim([-180 180]);
        
        % Plot Pitch (semplice e garantito)
        subplot(3,1,2);
        plot(timestamps, eulerAngles(:,2), 'g-');
        title('Pitch Angle');
        xlabel('Time (s)');
        ylabel('Degrees');
        grid on;
        ylim([-180 180]);
        
        % Plot Yaw (semplice e garantito)
        subplot(3,1,3);
        plot(timestamps, mod(eulerAngles(:,3), 360), 'b-'); % Yaw 0-360°
        title('Yaw Angle (0-360°)');
        xlabel('Time (s)');
        ylabel('Degrees');
        grid on;
        ylim([0 360]);
        
        disp('Grafici 2D generati con successo!');
        
    catch ME
        error(['Errore durante l''esecuzione: ' ME.message]);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function BNO055_Data_Plotter()
%     % Carica i dati
%     if exist('imu_data.mat', 'file')
%         load('imu_data.mat');
%     else
%         error('File imu_data.mat non trovato. Esegui prima Serial_Reader_BNO055.m');
%     end
% 
%     % Crea figure con subplot
%     figure('Name', 'BNO055 Euler Angles', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 700]);
% 
%     % Plot Roll
%     subplot(3,1,1);
%     plot(timestamps, eulerAngles(:,1)), grid on;
%     title('Roll Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot Pitch
%     subplot(3,1,2);
%     plot(timestamps, eulerAngles(:,2)), grid on;
%     title('Pitch Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot Yaw
%     subplot(3,1,3);
%     plot(timestamps, eulerAngles(:,3)), grid on;
%     title('Yaw Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot 3D Orientation (semplificato)
%     figure('Name', '3D Orientation', 'NumberTitle', 'off');
%     for i = 1:20:length(timestamps)
%         cla;
%         roll = eulerAngles(i,1);
%         pitch = eulerAngles(i,2);
%         yaw = eulerAngles(i,3);
% 
%         % Converti in radianti
%         roll_rad = deg2rad(roll);
%         pitch_rad = deg2rad(pitch);
%         yaw_rad = deg2rad(yaw);
% 
%         % Crea matrice di rotazione
%         R = rotz(yaw) * roty(pitch) * rotx(roll);
% 
%         % Disegna gli assi
%         axisLength = 0.5;
%         hold on;
%         quiver3(0,0,0, R(1,1),R(2,1),R(3,1), 'r', 'LineWidth', 2); % X
%         quiver3(0,0,0, R(1,2),R(2,2),R(3,2), 'g', 'LineWidth', 2); % Y
%         quiver3(0,0,0, R(1,3),R(2,3),R(3,3), 'b', 'LineWidth', 2); % Z
%         hold off;
% 
%         axis equal;
%         axis([-1 1 -1 1 -1 1]);
%         view(30,30);
%         title(sprintf('Orientamento 3D (t=%.1fs)', timestamps(i)));
%         legend('X', 'Y', 'Z');
%         drawnow;
%     end
% end
% 
% % Funzioni helper per matrici di rotazione
% function R = rotx(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [1 0 0;
%          0 cos(angle) -sin(angle);
%          0 sin(angle) cos(angle)];
% end
% 
% function R = roty(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [cos(angle) 0 sin(angle);
%          0 1 0;
%          -sin(angle) 0 cos(angle)];
% end
% 
% function R = rotz(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [cos(angle) -sin(angle) 0;
%          sin(angle) cos(angle) 0;
%          0 0 1];
% end

%%%%%%%%%%%%%



% function BNO055_Data_Plotter()
%     % Carica i dati
%     if exist('imu_data.mat', 'file')
%         load('imu_data.mat');
%     else
%         error('File imu_data.mat non trovato. Esegui prima Serial_Reader_BNO055.m');
%     end
% 
%     % Crea figure con subplot
%     figure('Name', 'BNO055 Euler Angles', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 700]);
% 
%     % Plot Roll
%     subplot(3,1,1);
%     plot(timestamps, eulerAngles(:,1), grid on;
%     title('Roll Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot Pitch
%     subplot(3,1,2);
%     plot(timestamps, eulerAngles(:,2), grid on;
%     title('Pitch Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot Yaw
%     subplot(3,1,3);
%     plot(timestamps, eulerAngles(:,3), grid on;
%     title('Yaw Angle');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     ylim([-180 180]);
% 
%     % Plot 3D Orientation (semplificato)
%     figure('Name', '3D Orientation', 'NumberTitle', 'off');
%     for i = 1:20:length(timestamps)
%         cla;
%         roll = eulerAngles(i,1);
%         pitch = eulerAngles(i,2);
%         yaw = eulerAngles(i,3);
% 
%         % Converti in radianti
%         roll_rad = deg2rad(roll);
%         pitch_rad = deg2rad(pitch);
%         yaw_rad = deg2rad(yaw);
% 
%         % Crea matrice di rotazione
%         R = rotz(yaw) * roty(pitch) * rotx(roll);
% 
%         % Disegna gli assi
%         axisLength = 0.5;
%         hold on;
%         quiver3(0,0,0, R(1,1),R(2,1),R(3,1), 'r', 'LineWidth', 2); % X
%         quiver3(0,0,0, R(1,2),R(2,2),R(3,2), 'g', 'LineWidth', 2); % Y
%         quiver3(0,0,0, R(1,3),R(2,3),R(3,3), 'b', 'LineWidth', 2); % Z
%         hold off;
% 
%         axis equal;
%         axis([-1 1 -1 1 -1 1]);
%         view(30,30);
%         title(sprintf('Orientamento 3D (t=%.1fs)', timestamps(i)));
%         legend('X', 'Y', 'Z');
%         drawnow;
%     end
% end
% 
% % Funzioni helper per matrici di rotazione
% function R = rotx(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [1 0 0;
%          0 cos(angle) -sin(angle);
%          0 sin(angle) cos(angle)];
% end
% 
% function R = roty(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [cos(angle) 0 sin(angle);
%          0 1 0;
%          -sin(angle) 0 cos(angle)];
% end
% 
% function R = rotz(angle_deg)
%     angle = deg2rad(angle_deg);
%     R = [cos(angle) -sin(angle) 0;
%          sin(angle) cos(angle) 0;
%          0 0 1];
% end