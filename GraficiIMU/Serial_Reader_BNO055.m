function Serial_Reader_BNO055()
    % Configurazione porta seriale
    port = 'COM4'; % Modifica con la tua porta COM
    baudRate = 115200; % Deve corrispondere a quello nella tua USART3
    
    % Crea l'oggetto seriale
    try
        s = serialport(port, baudRate);
        configureTerminator(s, "LF"); % Configura il terminatore di linea
        disp(['Connesso alla porta seriale: ' port]);
    catch e
        error(['Errore connessione seriale: ' e.message]);
    end
    
    % Parametri acquisizione
    duration = 60; % Durata acquisizione in secondi
    startTime = datetime('now');
    
    % Inizializza array per i dati
    maxSamples = duration * 10; % Stima massima (1Hz dal tuo codice)
    timestamps = zeros(maxSamples, 1);
    eulerAngles = zeros(maxSamples, 3); % [Roll, Pitch, Yaw]
    sampleCount = 0;
    
    % File per salvare i dati
    dataFile = fopen('imu_data.csv', 'w');
    fprintf(dataFile, 'Timestamp,Roll,Pitch,Yaw\n');
    
    % Loop di acquisizione
    disp('Acquisizione dati in corso...');
    flush(s);
    
    while seconds(datetime('now') - startTime) < duration
        % Leggi una riga
        dataLine = readline(s);
        
        % Parsing dei dati (formato: "Roll: X.XX, Pitch: X.XX, Yaw: X.XX")
        try
            dataParts = strsplit(dataLine, ',');
            
            roll = sscanf(dataParts{1}, 'Roll: %f');
            pitch = sscanf(dataParts{2}, ' Pitch: %f');
            yaw = sscanf(dataParts{3}, ' Yaw: %f');
            
            sampleCount = sampleCount + 1;
            timestamps(sampleCount) = seconds(datetime('now') - startTime);
            eulerAngles(sampleCount,:) = [roll, pitch, yaw];
            
            % Salva su file
            fprintf(dataFile, '%.3f,%.2f,%.2f,%.2f\n', ...
                   timestamps(sampleCount), roll, pitch, yaw);
            
        catch
            warning('Errore parsing linea: %s', dataLine);
        end
    end
    
    % Pulisci gli array dai campioni non utilizzati
    timestamps = timestamps(1:sampleCount);
    eulerAngles = eulerAngles(1:sampleCount,:);
    
    % Chiudi le risorse
    fclose(dataFile);
    clear s;
    
    % Salva i dati in formato .mat
    save('imu_data.mat', 'timestamps', 'eulerAngles');
    disp('Acquisizione completata. Dati salvati in imu_data.mat e imu_data.csv');
end