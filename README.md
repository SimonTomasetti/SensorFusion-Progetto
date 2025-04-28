# Sensor Fusion Project - Localization with UWB, IMU, and Encoder

Questo progetto riguarda la realizzazione di un sistema di localizzazione basato su Sensor Fusion utilizzando:

- Modulo UWB Decawave DWM1001
- IMU Bosch BNO055
- Encoder capacitivo AMT102
- Microcontrollore STM32H755ZI-Q

Il codice è stato sviluppato in:
- **STM32CubeIDE** (linguaggio C)
- **MATLAB/Simulink** (Creazione Grafici)

## Funzionalità principali
- Comunicazione seriale UART tra STM32 e DWM1001 (protocollo TLV)
- Implementazione di un filtro di Kalman custom per la fusione dei dati
- Predisposizione per estensione con dati da IMU e Encoder
- Compatibilità con piattaforme di visualizzazione come Putty

## Documentazione
Le fonti principali utilizzate per il progetto sono:
- Relazione di Alessandro Di Biase
- Documentazione ufficiale Qorvo per DWM1001
- Datasheet Bosch Sensortec (BNO055)
- Datasheet Encoder AMT102
- Lezioni del corso "Laboratorio di Automazione" (Prof. Andrea Bonci)

## Autore
- Simon Tomasetti, Dennis Sgreccia, Francesco Iurilli  
- Progetto universitario, Università Politecnica delle Marche
