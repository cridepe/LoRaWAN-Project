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

| Colonna 1 | Colonna 2 | Colonna 3 |
|-----------|-----------|-----------|
| Riga 1    | Dato A    | Dato B    |
| Riga 2    | Dato C    | Dato D    |


