# LoRaWAN-Project
Questo progetto si pone l'obbiettivo di realizzare un sistema di videosorveglianza per i lupi per la sicurezza dei pascoli.

Le parti che verranno trattate in questo progetto sono:
- Creazione di un sistema di sensori che attiva la fotocamera;
- Scatto di una foto;
- Salvataggio, compressione e suddivisione in pacchetti della foto;
- Invio dei paccheti su server TTN;
- Ricezione dei pacchetti da TTN su Node-RED;
- Unione dei pacchetti su Node-RED, conversione in base64 e stampa della foto su dashboard.

## Programmazione su STM32CubeIDE

Per scrivere i codici sono state usate varie nucleo tra cui la Nucleo-L476 e la Nucleo-F207, l'ideale sarebbe di utilizzare la Nucleo-F407 per realizzare il programma finale.

### Invio Pacchetti con LoRaWAN

Come prima cosa ci siamo occupati di implementare la parte relativa all'invio dei pacchetti con LoRaWAN.
Per fare ciò abbiamo utilizzato un modulo rfm95 con la libreria rfm95.c, e abbiamo impostato la Nucleo-L476 come segue:


| Pin Name | GPIO Output | GPIO Mode | GPIO Pull-Up/Pull-Down | Maximum Output Speed | User Label |
|----------|-------------|-----------|------------------------|----------------------|------------|
| PB0    | High    | Output Push Pull     | Pull-Up | Low | CS |
| PB1    | High    | Output Open Drain     | No Pull-Up and no Pull-Down | Low | RST |
| PA7    | n/a    | External Interrupt     | No Pull-Up and no Pull-Down | n/a | DIO0 |
| PA6    | n/a    | External Interrupt     | No Pull-Up and no Pull-Down | n/a | DIO1 |
| PA5    | n/a    | External Interrupt     | No Pull-Up and no Pull-Down | n/a | DIO5 |

- LPTMI: Counts Internal CLock Events;
- RTC: Activate Clock Sources;
- SPI2: Full-Duplex Master, Maximum Output Speed -> High;
- USART2: Asynchronous;
- RNG: Activated.

![immagine](https://github.com/user-attachments/assets/7bf237f8-3d66-470c-89d4-9a5ad027fba0)
![immagine](https://github.com/user-attachments/assets/42940b34-432f-4670-8935-b13663c42931)

### Configurazione TTN

Il server TTN ci servirà per leggere i dati che vengono inviati tramite LoRaWAN. 
Dopo averlo configuaro come segue: 

![immagine](https://github.com/user-attachments/assets/2cfe8892-9ee5-48cd-b9a8-915738db4afd)
![immagine](https://github.com/user-attachments/assets/900a1600-ddfa-4530-be6b-e6d340542a00)

Ne ricaviamo il DeviceAdress, la NwkSKey e l'AppSKey che ci serviranno nel codice per stabilire una connessione con il server.
Dopo di che, modifichiamo il payload in modo da riucire a salvare i dati esadecimali in una variabile che ci servirà poi su Node-RED.

### Configurazione Node-RED

Node-RED ci servirà per ricevere i pacchetti di dati che arrivano su TTN, unirli assieme e stampare sulla dashboard la foto ricostruita. Ci basterà importare il file Node-RED FLow.

### Scatto Foto OV2640

Per impostare la Nucleo-L207 per usare la fotocamera abbiamo seguito questa pagina https://github.com/SimpleMethod/STM32-OV2640?tab=readme-ov-file e usato la sua libreria ov2640.c, con l'aggiunta di qualche impostazione tra cui il pin GPIO del reset della camera con output level impostato HIGH così come il PWDN. Inoltre va impostata la maximum output speed dei pin DCMI su HIGH.

### Sensori
Radar: RCWL0512
PIR: HCSR 501




