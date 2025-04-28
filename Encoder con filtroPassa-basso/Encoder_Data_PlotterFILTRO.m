function Encoder_Data_Plotter()
    % Configurazione iniziale
    close all; clear; clc;
    
    try
        % Caricamento dati
        if ~exist('encoder_data.csv', 'file')
            error('File encoder_data.csv non trovato');
        end
        
        data = readtable('encoder_data.csv');
        timestamps = data.Timestamp;
        angles = data.Angle;
        speeds_rpm = data.Speed_RPM;
        filtered_speeds = data.Filtered_Speed_RPM;
        
        % -----------------------------------------------------------
        % 1. GRAFICO POSIZIONE ANGOLARE
        % -----------------------------------------------------------
        figure('Name', 'ATM102V Encoder Data - Filtered', 'Position', [100, 100, 1000, 800]);
        
        subplot(3,1,1);
        plot(timestamps, angles, 'b', 'LineWidth', 1.5);
        title('Posizione Angolare');
        ylabel('Gradi (°)');
        grid on;
        ylim([min(angles)-5 max(angles)+5]);
        
        % -----------------------------------------------------------
        % 2. GRAFICO VELOCITA' ORIGINALE
        % -----------------------------------------------------------
        subplot(3,1,2);
        plot(timestamps, speeds_rpm, 'r', 'LineWidth', 1.5);
        title('Velocità Rotazionale (Originale)');
        ylabel('RPM');
        grid on;
        
        % -----------------------------------------------------------
        % 3. GRAFICO VELOCITA' FILTRATA
        % -----------------------------------------------------------
        subplot(3,1,3);
        plot(timestamps, filtered_speeds, 'g', 'LineWidth', 1.5);
        title('Velocità Rotazionale (Filtrata)');
        ylabel('RPM');
        xlabel('Tempo (s)');
        grid on;
        
    catch ME
        errordlg(['Errore: ' ME.message], 'File Error');
    end
end