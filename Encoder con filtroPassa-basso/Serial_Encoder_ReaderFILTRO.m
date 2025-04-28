function Serial_Encoder_Reader()
    try
        % 1. Configurazione iniziale
        close all;
        clearvars -except s;
        clc;

        % 2. Creazione file con percorso assoluto
        [filepath,~,~] = fileparts(mfilename('fullpath'));
        csv_path = fullfile(filepath, 'encoder_data.csv');
        dataFile = fopen(csv_path, 'w');

        if dataFile == -1
            error('Impossibile creare il file di output');
        end

        fprintf(dataFile, 'Timestamp,Angle,Speed_RPM,Filtered_Speed_RPM\n');

        % 3. Configurazione seriale con timeout
        if ~exist('s', 'var') || ~isvalid(s)
            s = serialport('COM4', 115200, 'Timeout', 10);
            configureTerminator(s, "LF");
        end

        flush(s);

        % 4. NUOVO pattern regex per gestire caratteri speciali
        pattern = 'Angolo:\s*(?<angle>[\d\.]+).*Velocit.*:\s*(?<speed>[\d\.]+).*Velocit.*filtrata:\s*(?<filtered>[\d\.]+)';

        disp('Acquisizione iniziata. Premere Ctrl+C per terminare...');
        disp('Tempo massimo: 60 secondi');

        % 5. Acquisizione dati con timer
        startTime = tic;
        while toc(startTime) < 10  % 60 secondi
            try
                if s.NumBytesAvailable > 0
                    dataLine = readline(s);
                    disp(">> Ricevuto:");
                    disp(dataLine);

                    % 6. Parsing con gestione caratteri speciali
                    cleanLine = strrep(dataLine, 'Â°', '°');
                    cleanLine = strrep(cleanLine, 'Ã ', 'à');
                    parsedData = regexp(cleanLine, pattern, 'names');

                    if ~isempty(parsedData)
                        fprintf(dataFile, '%.3f,%.2f,%.2f,%.2f\n', ...
                            toc(startTime), ...
                            str2double(parsedData.angle), ...
                            str2double(parsedData.speed), ...
                            str2double(parsedData.filtered));
                    else
                        warning('Dati non riconosciuti: %s', strtrim(dataLine));
                    end
                else
                    pause(0.01); % Piccola pausa per ridurre CPU usage
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:serialport:readline:operationTerminated')
                    break;
                end
                warning('Errore durante la lettura: %s', ME.message);
            end
        end

    catch ME
        disp(['Errore critico: ' ME.message]);
    end

    % 7. Chiusura sicura risorse
    if exist('dataFile', 'var') && dataFile ~= -1
        fclose(dataFile);
        disp(['Dati salvati in: ' csv_path]);
    end

    if exist('s', 'var') && isvalid(s)
        delete(s);
        clear s;
    end
    
    disp('Acquisizione completata');
end