# Biblioteca nRF24Communication
 Abaixo tem-se a documentação da biblioteca nRF24Communication, onde usamos as funções da biblioteca [Biblioteca RF24](http://tmrh20.github.io/RF24/) aplicado a robótica.

# Um exemplo de código
No código a seguir você configura um nrf que recebe mensagens: 

```cpp
#include <nRF24Communication.h>                         // incluindo a lib

bool justUpdated = false;
nRF24Communication radio = nRF24Communication(A0,10);   // parâmetros pinCE,pinCS - portas do nRF24L01


void setup(){
    radio.setup(1, 400);                                // configura o radio com id 1 e tempo de espera máximo entre uma mensagem e outra, fica configurado para receber mensagem
    Serial.begin(9600);

}


void loop(){
    justUpdated = radio.updateInfo();                  // verifica se tem mensagem nova    

    if(justUpdated){
        Serial.println(radio.getLeftMotorSpeed());      
        Serial.println(radio.getRightMotorSpeed());
    }

}

```

# Fluxo Do Código
![Fluxo do código](../imgs/robo_comm.png)

# Tipos de Mensagens
Atualmente temos **seis** tipos de mensagens mais **um** tipo básico usado para indentificação  geral das mensagens.

**>> TypeSpeed**

Tipo de mensagem que envia para o robô as velocidades dos seus motores e pode pedir para o robô retornar algum dado através do campo flags.

|   Campos  |     Quantidade de Bits|        Função                                                     			| 
|-----------|-----------------------|-------------------------------------------------------------------------|
|typeMsg	  |`4 bits`           	  |Indentificar o tipo de mensagem 		                                      |
|id         |`4 bits`            	  |Indentificar para que robô a mensagem está sendo enviada           			|
|leftSpeed  |`8 bits`				        | Velocidade do motor esquerdo                                            |
|rightSpeed |`8 bits`				        | Velocidade do motor direito                                             |
|flags 		  |`8 bits`				        | Tipos de retorno que o robô pode devolver                               |

**>> TypePositions**

|   Campos  |     Quantidade de Bits|        Função              			       |    
|-----------|-----------------------|---------------------------------------|
|typeMsg	   |`4 bits`           	   |Indentificar o tipo de mensagem 		|
|id         |`4 bits`            	|Indentificar para que robô a mensagem está sendo enviada           			|
|curPosX  	|`8 bits`				| Posição X atual do robô|
|curPosY 	|`8 bits`				| Posição Y atual do robô|
|curAngle 	|`16 bits`				| Ângulo atual do robô|
|objPosX  	|`8 bits`				| Posição X objetivo do robô|
|objPosY 	|`8 bits`				| Posição Y objetivo do robô|
|objAngle 	|`16 bits`				| Ângulo objetivo do robô|
|flags 		|`8 bits`				| Tipos de retorno que o robô pode devolver|

**>> TypePID**

|   Campos  |     Quantidade de Bits|        Função              			| 
|-----------|-----------------------|---------------------------------------|
|typeMsg	|`4 bits`           	|Indentificar o tipo de mensagem 		|
|id         |`4 bits`            	|Indentificar para que robô a mensagem está sendo enviada           			|
|kp  	|`16 bits`				| Constante proporcional|
|ki 	|`16 bits`				| Constante integral|
|kd 	|`16 bits`				| Constante derivativo|
|alpha  	|`16 bits`				| Aceleração|
|flags 		|`8 bits`				| Tipos de retorno que o robô pode devolver|

**>> TypeRetPosition**

|   Campos  |     Quantidade de Bits|        Função              			| 
|-----------|-----------------------|---------------------------------------|
|typeMsg	|`4 bits`           	|Indentificar o tipo de mensagem 		|
|id         |`4 bits`            	|Indentificar para que robô a mensagem está sendo enviada           			|
|curPosX  	|`8 bits`				| Posição X atual do robô|
|curPosY 	|`8 bits`				| Posição Y atual do robô|
|curAngle 	|`16 bits`				| Ângulo atual do robô|
|end 		|`8 bits`				| Final de mensagem |

**>> TypeRetBattery**

|   Campos  |     Quantidade de Bits|        Função              			| 
|-----------|-----------------------|---------------------------------------|
|typeMsg	|`4 bits`           	|Indentificar o tipo de mensagem 		|
|id         |`4 bits`            	|Indentificar para que robô a mensagem está sendo enviada           			|
|batteryLevel  	|`8 bits`				| Nível da Bateria|
|curAngle 	|`16 bits`				| Ângulo atual do robô|
|end 		|`8 bits`				| Final de mensagem |

# Construção dos Tipos
Nessa seção explicaremos como é feita a construção de um tipo e como você poderá criar um novo tipo.

Para começar usemos como exemplo o tipo **TypeSpeed**:
```cpp
 typedef struct typeSpeed{
    uint8_t typeMsg:4;
    uint8_t id:4;
    int8_t leftSpeed:8;
    int8_t rightSpeed:8;
    uint8_t flags:8;
  } __attribute__((packed)) TypeSpeed;

  typedef union msgSpeed{
    unsigned char encoded[4];
    TypeSpeed decoded;
 } MsgSpeed;
``` 
Inicialmente criamos uma struct e usamos o conceito de [Bit Field](http://en.cppreference.com/w/cpp/language/bit_field) onde conseguimos declarar um tipo com uma quantidade restrita de bits, portanto para cada variavél existente na struct é determinado o número de bits que vamos usar nesta e por fim a struct funciona como um grande tipo de dados com uma quantidade de bits. A flag "__attribute__((packed))" evita com que o gcc complete bits, ou seja, dessa forma dizemos ao compilador que ele não precisa preecher com bits aleatórios pois queremos aquele tipo específico.

Portanto cabe agora explicar o uso do union, como o [nrf24l01](http://tmrh20.github.io/RF24/classRF24.html#a4cd4c198a47704db20b6b5cf0731cd58) na biblioteca base que usamos para envio usa-se ponteiro o que fazemos nesso ponto é transformar nosso tipo recém criado em um ponteiro através da união. Para isso declaramos dentro do union uma variável com o tipo da struct que criamos e uma variavél que é um vetor com a quantidade de bytes que usamos.Assim, quando modificamos um o outro é modificado consequentemente. 

```cpp
 .
 .
 .
 MsgSpeed M1;
 
 M1.decoded.typeMsg = 10; 
 M1.decoded.id = 2;
 
 // Isso é equivalente ao que eu fiza acima:
 M1.encoded [1] = 162;
 
 // 10 = 1010
 // 02 = 0010
 // 162 = 10100010 | Em um byte temos o typeMsg+id
 .
 .
 .
 
```

# Funções
Atualmente, temos as seguintes funções na biblioteca de comunicação:

**>> nRF24Communication(int pinCE, int pinCSN)**

Construtor da classe, recebe como parâmetros o pino do CE e do CSN.
Inicializa as velocidades dos motores como 0, e define flags como falsas.
Define os pinos do CE e CSN a partir dos parâmetros de entrada.

**>> setup(int roboSwitches, unsigned long watchdog_millis_time)**

Função de setar os parâmetros iniciais.
Chama uma instância de RF24.
Atualiza o ID do robô baseado no switch (updateRobotId()).
Atualiza o momento da última recepção de informações (momento atual), e define essa atualização como true.
Atualiza o tempo do watchdog a partir do parâmetro de entrada.
Chama a função para configurar a comunicação RF(_configure()).

**>> updateInfo()**

Começa a ouvir pelo NRF.
Caso o chip esteja conectado, enquanto tiver informação disponível, lê as informações recebidas pelo NRF e as salva em uma variável codificada. Também salva o tipo de mensagem.
Se a mensagem for para esse robô, dependendo do tipo de mensagem, atualiza as informações do robô: velocidade (dos dois motores), posição (coordenadas atuais do robô e da bola), e PID (kp, kd, ki e alpha).
Atualiza também o tempo de recepção da última informação.
Caso o chip não esteja conectado, pára de ouvir para reconfigurar, e na próxima vez tenta novamente.

**>> sendInfo(const void * buf,uint8_t len)**

Para de ouvir, e manda informação pelo NRF.

**>> getLeftMotorSpeed()**

Retorna a velocidade do motor esquerdo.

**>> getRightMotorSpeed()**

Retorna a velocidade do motor direito.

**>> getCurX()**

Retorna a coordenada X atual do robô.

**>> getCurY()**

Retorna a coordenada Y atual do robô.

**>> getCurAngle()**

Retorna o ângulo atual do robô.

**>> getObjX()**

Retorna a coordenada X atual da bola.

**>> getObjY()**

Retorna a coordenada Y atual da bola.

**>> getObjAngle()**

Retorna o ângulo atual da bola.

**>> shallSendBatteryLevel()**

Retorna o booleano que diz deve mandar ou não o nível da bateria, de acordo com as últimas flags recebidas.

**>> updateRobotId(int roboSwID)**

Atualiza o ID do robô a partir da informação do switch.

**>> getRobotId()**

Retorna o ID do robô.

**>> sendBatteryLevel(float battery)**

Pega o nível de bateria do robô e manda para o PC através do NRF.

**>> sendPositions(int PosX,int PosY, float Angle)**

Pega as informações da posição do robô.

**>> timeupWatchdog()**

Diz se o tempo que passou desde a última atualização passou o tempo limite estabelecido.

**>> getTypeOfMessage()**

Retorna o tipo de mensagem.

**>> getKP()**

Retorna o parâmetro kp do PID.

**>> getKD()**

Retorna o parâmetro kd do PID.

**>> getAlpha()**

Retorna o parâmetro alpha do PID.

# Funções Privadas
As funções privadas da biblioteca de comunicação são:

**>> _configure()**

Configura os parâmetros da comunicação RF, utilizando as funções da biblioteca RF24: setPALevel, setChannel, setAutoAck, setPayloadSize, openWritingPipe, openReadingPipe.
Começa a receber pelo NRF (_receive());

**>> _receive()**

Começa a ouvir pelo NRF, através da própria função do RF24: startListening.


**>> _send()**

Pára de ouvir pelo NRF para mandar, através da função do RF24: stopListening.

**>> _setFlags(uint8_t value)**

Atualiza as flags de mandar nível da bateria, mandar posições, “kick” e “dribbler”.

**>> _setFlagsToFalse()**

Coloca todas as flags para “false”.
