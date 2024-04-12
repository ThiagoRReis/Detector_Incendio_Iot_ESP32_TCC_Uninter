/* SENSOR DE FUMAÇA, GASES INFLAMÁVEIS, TEMPERATURA E BOTÃO AJUDA IOT COM ESP32 DE 30 PINOS (DEVKIT)
 * MICROCONTROLADOR: ESP32 WROOM32-WIFI 30 PINOS
 * IDE: ARDUINO IDE 2.3.2
 * TCC UNINTER: ENGENHARIA DA COMPUTAÇÃO
 * AUTOR: THIAGO RODRIGUES REIS
 * VERSÃO: 1.0
 * DATA: 11/03/2024
 */



// BIBLIOTECAS
#include <WiFiManager.h>  // BIBLIOTECA PARA GERENCIAR A CONEXÃO WIFI
#include <HTTPClient.h>   // BIBLIOTECA PARA GERENCIAR O ENVIO DA MENSAGEM POR INTERMÉDIO DA API CALLMEBOT
#include <esp_task_wdt.h> // BIBLIOTECA PARA UTILIZAÇÃO DO WATCHDOG



// DEFINIÇÃO DOS PINOS OU GPIOS DO NodeMCU-ESP32 WROOM
#define entradaMq2         18  // DEFINE O PINO OU GPIO 18 COMO ENTRADA DO SINAL ADVINDO DO SENSOR MQ2
#define saidaFogoPtencia   19  // DEFINE O PINO OU GPIO 19 COMO SAÍDA PARA ATIVAR A PARTE DE POTÊNCIA EM CASO DE FOGO
#define saidaBuzzerLed     21  // DEFINE O PINO OU GPIO 21 COMO SAÍDA PARA ATIVAR BUZZER E LED INTERNO DA PLACA EM CASO DE FOGO
#define entradaBtnAjuda    13  // DEFINE O PINO OU GPIO 13 COMO ENTRADA DO SINAL ADVINDO DO BOTÃO DE AJUDA
#define saidaAjudaPotencia 27  // DEFINE O PINO OU GPIO 27 COMO SAÍDA PARA ATIVAR A PARTE DE POTÊNCIA EM CASO DE PEDIDO DE AJUDA
#define entradaNTC         33  // DEFINE O PINO OU GPIO 33 COMO ENTRADA DO SINAL ADVINDO DO NTC



// DEFINIÇÃO DAS URLS
const String urlMensagemFogo  = "https://api.callmebot.com/whatsapp.php?phone=557381591887&text=ALERTA+PARA+FOGO!&apikey=1590661";        // MENSAGEM PARA FOGO
const String urlMensagemAjuda = "https://api.callmebot.com/whatsapp.php?phone=557381591887&text=ALERTA+PEDIDO+DE+AJUDA!&apikey=1590661";  // MENSAGEM PARA AJUDA



// VARIÁVEIS DE USO GERAL
bool mq2Acionado               = false;  // VARIÁVEL PARA CONTROLAR A CHAMADA NA FUNÇÃO monitoraMq2(), ASSIM SÓ SERÁ ENVIADA MENSAGEM A CADA VALOR DE TEMPO IGUAL A VARIÁVEL tempoParaEnvio 
bool btnAjudaAcionado          = false;  // VARIÁVEL PARA CONTROLAR A CHAMADA NA FUNÇÃO monitoraBtnAjuda(), ASSIM SÓ SERÁ ENVIADA MENSAGEM A CADA VALOR DE TEMPO IGUAL A VARIÁVEL tempoParaEnvio
bool ntcFoiLido                = false;  // VARIÁVEL PARA CONTROLAR A CHAMADA NA FUNÇÃO leituraDoNTC(), ASSIM O NTC SÓ SERÁ LIDO A CADA VALOR DE TEMPO IGUAL A VARIÁVEL tempoParaLeituraNTC
bool ntcMonitorado             = false;  // VARIÁVEL PARA CONTROLAR A CHAMADA NA FUNÇÃO monitoraNTC(), ASSIM SÓ SERÁ ENVIADA MENSAGEM A CADA VALOR DE TEMPO IGUAL A VARIÁVEL tempoParaEnvio
unsigned int tempoParaEnvio    = 60000;  // VARIÁVEL PARA CONTROLAR O TEMPO ENTRE OS ENVIOS DAS MENSAGENS
long tempoAnteriorMq2          = 0;      // VARIÁVEL PARA GUARDAR O TEMPO QUANDO A FUNÇÃO monitoraMq2() FOR CHAMADA
long tempoFuturoMq2            = 0;      // VARIÁVEL PARA VERIFICAR SE JÁ PASSOU tempoParaEnvio EM SEGUNDOS DESDE DE QUE A FUNÇÃO monitoraMq2() FOI CHAMADA
long tempoAnteriorBtnAjuda     = 0;      // VARIÁVEL PARA GUARDAR O TEMPO QUANDO A FUNÇÃO monitoraBtnAjuda() FOR CHAMADA
long tempoFuturoBtnAjuda       = 0;      // VARIÁVEL PARA VERIFICAR SE JÁ PASSOU tempoParaEnvio EM SEGUNDOS  DESDE DE QUE A FUNÇÃO monitoraBtnAjuda() FOI CHAMADA
long tempoAnteriorMonitoraNTC  = 0;      // VARIÁVEL PARA GUARDAR O TEMPO QUANDO A FUNÇÃO monitoraNTC() FOR CHAMADA      
long tempoFuturoMonitoraNTC    = 0;      // VARIÁVEL PARA VERIFICAR SE JÁ PASSOU tempoParaEnvio EM SEGUNDOS DESDE DE QUE A FUNÇÃO monitoraNTC() FOI CHAMADA
long tempoAnteriorNTC          = 0;      // VARIÁVEL PARA GUARDAR O TEMPO QUANDO A FUNÇÃO leituraDoNTC() FOR CHAMADA 
long tempoFuturoNTC            = 0;      // VARIÁVEL PARA VERIFICAR SE JÁ PASSOU tempoParaLeituraNTC EM SEGUNDOS DESDE DE QUE A FUNÇÃO leituraDoNTC() FOI CHAMADA
int tempoParaLeituraNTC        = 4000;   // VARIÁVEL PARA CONTROLAR O TEMPO ENTRE AS LEITURAS DO NTC   
int temperaturaAlertaNTC       = 60;     // VARIÁVEL PARA GUARDAR A TEMPERATURA PARA ALERTA EM CASO DE FOGO
char qtdMedidasParaNTC         = 5;      // VARIÁVEL PARA DETERMINAR A QUANTIDADE DE LEITURAS DO NTC PARA FORMAR TEMPERATURA MÉDIA



// CONSTANTES PARA LEITURA DO NTC
float Vs        = 3.3;                      // TENSÃO DE SAÍDA DO ESP32
float R1        = 10000;                    // RESISTOR UTILIZADO NO DIVISOR DE TENSÃO
float Beta      = 3950;                     // VALOR DE BETA
float To        = 298.15;                   // VALOR EM KELVIN REFERENTE A 25° CELSIUS
float Ro        = 10000;                    // RESISTÊNCIA DO NTC A 25° CELSIUS
float adcMax    = 4095.0;                   // RESOLUÇÃO MÁXIMA DO ADC



// VARIÁVEL GLOBAL PARA GUARDAR O VALOR DA TEMPERATURA LIDA COM NTC JÁ CONVERTIDA EM GRAUS CELSIUS
float Tc     = 0;                     



// INÍCIO SETUP
void setup() 
{
//---------------------------------------------------------CONFIGURAÇÃO PARA CONEXÃO COM WIFI-----------------------------------------------------------------------------     
  WiFi.mode(WIFI_STA);                      // MODO DEFINIDO EXPLICITAMENTE, ESP PADRÃO PARA STA + AP   
  Serial.begin(115200);                     // INICADA A SERIAL PARA VERIFICAR STATUS      
  WiFiManager wm;                           // OBJETO wm PARA USAR AS FUNÇÕES DA BIBLIOTECA WifiManager

  //wm.resetSettings();                     // COMANDO PARA RESETAR AS CONFIGURAÇÕES DO WIFI (OPCIONAL)

  wm.setConfigPortalTimeout(60);             // TEMPO EM ESPERA CASO NÃO CONECTE AO WIFI... REINICIA O PROCESSO TENTANDO CONECTAR AO WIFI NOVAMENTE
  bool res;                                  // VARIÁVEL PARA ARMAZENAR O RETORNO DA CONEXÃO WIFI LOCAL DO ESP32

  res = wm.autoConnect("ESP_32","12345678"); // NOME DA REDE E SENHA GERADA PELO ESP32, ONDE SERÃO INSERIDAS AS CREDENCIAIS DA REDE WIFI (IP PADRÃO: 192.168.4.1)

  if(!res) {
    Serial.println("Conexão falhou!");
    ESP.restart();                           // SE CONEXÃO FALHAR REINICIA PARA TENTAR CONECTAR NOVAMENTE
  } 
  else {
    Serial.println("Conectado ao Wifi!");
    Serial.println("Endereco IP: ");
    Serial.println(WiFi.localIP());          // INFORMA O IP
  }
  
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------CONFIGURAÇÃO DOS PINOS OU GPIOS-----------------------------------------------------------------------------------
  pinMode(entradaMq2, INPUT);                // DEFINE entradaMq2 COMO ENTRADA
  pinMode(saidaFogoPtencia, OUTPUT);         // DEFINE saidaFogoPtencia COMO SAÍDA
  pinMode(saidaBuzzerLed, OUTPUT);           // DEFINE saidaBuzzerLed COMO SAÍDA


  pinMode(entradaBtnAjuda, INPUT_PULLDOWN);  // DEFINE entradaBtnAjuda COMO ENTRADA COM RESISTOR INTERNO DE PULLDOWN
  pinMode(saidaAjudaPotencia, OUTPUT);       // DEFINE saidaAjudaPotencia COMO SAÍDA

  pinMode(entradaNTC, INPUT);      // DEFINE entradaNTC COMO ENTRADA COM RESISTOR INTERNO DE PULLDOWN


  digitalWrite(saidaFogoPtencia, LOW);       // INICIA saidaFogoPtencia EM NÍVEL BAIXO (DESLIGADO)
  digitalWrite(saidaBuzzerLed, LOW);         // INICIA saidaBuzzerLed EM NÍVEL BAIXO (DESLIGADO)
  digitalWrite(saidaAjudaPotencia, LOW);     // INICIA saidaAjudaPotencia EM NÍVEL BAIXO (DESLIGADO)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------CONFIGURAÇÃO DO WATCHDOG------------------------------------------------------------------------------------------------
  esp_task_wdt_init(30, true); // INICIALIZA O WATCHDOG COM TEMPO DE ESPERA PARA RESET DE 15 SEGUNDOS
  esp_task_wdt_add(NULL);      // ASSINATURA PARA ALGUMA TAREFA (NESTE CASO NÃO EXISTE TAREFA)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
}
// FIM SETUP



// INÍCIO DO LOOP
void loop() {

  esp_task_wdt_reset(); // FUNÇÃO QUE IRÁ RESETAR O ESP32 CASO DEMORE MAIS DE 15 SEGUNDOS EM IR DO INÍCIO AO FINAL DO LOOP

//--------------------------------------------------------LEITURA DO NTC E GERENCIAMENTO DAS AÇÕES-------------------------------------------------------------------------
  // FUNÇÃO QUE EFETUA A LEITURA DO NTC E INFORMA A TEMPERATURA EM GRAUS CELSIUS
  leituraDoNTC();
  
  // RESETA A VARIÁVEL ntcFoiLido PARA PODER EFETUAR NOVA LEITURA DO NTC
  if(ntcFoiLido == true){
    tempoFuturoNTC = millis();
    if((tempoFuturoNTC - tempoAnteriorNTC) == tempoParaLeituraNTC){
      ntcFoiLido = false;
    }
  }

  // FUNÇÃO QUE MONITORA A TEMPERATURA DO NTC E GERENCIA AÇÕES
  monitoraNTC();

  // RESETA A VARIÁVEL ntcMonitorado PARA PODER ATUAR EM CASO DA TEMPERATURA SER MAIOR QUE A CONFIGURADA
  if(ntcMonitorado == true){
    tempoFuturoMonitoraNTC = millis();
    if((tempoFuturoMonitoraNTC - tempoAnteriorMonitoraNTC) == tempoParaEnvio){
      ntcMonitorado = false;
    }
  }  
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 



//---------------------------------------------------------LEITURA DO SENSOR MQ2 E GERENCIAMENTO DAS AÇÕES------------------------------------------------------------------
  // FUNÇÃO PARA MONITORAR O SENSOR MQ2 E GERENCIAR AÇÕES
  monitoraMq2();        

  // RESETA A VARIÁVEL mq2Acionado PARA O SENSOR SER ACIONADO NOVAMENTE
  if(mq2Acionado == true){
    tempoFuturoMq2 = millis();
    if((tempoFuturoMq2 - tempoAnteriorMq2) == tempoParaEnvio){
      mq2Acionado = false;
    }
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------LEITURA DO BOTÃOE GERENCIAMENTO DAS AÇÕES--------------------------------------------------------------------
  // FUNÇÃO PARA MONITORAR O BOTÃO DE AJUDA E GERENCIAR AÇÕES   
  monitoraBtnAjuda();    

  // RESETA A VARIÁVEL btnAjudaAcionado PARA O BOTÃO SER ACIONADO NOVAMENTE
  if(btnAjudaAcionado == true){
    tempoFuturoBtnAjuda = millis();
    if((tempoFuturoBtnAjuda - tempoAnteriorBtnAjuda) == tempoParaEnvio){
      btnAjudaAcionado = false;
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// FIM DO LOOP



//-------------------------------------------------------------------------FUNÇÕES DIVERSAS-------------------------------------------------------------------------------


//----------------------------------------------------FUNÇÃO PARA ENVIAR MENSAGEM CASO SENSOR MQ2 ESTEJA ATIVADO----------------------------------------------------------
void enviaMensagemFogo(){
                  
  int respostaCode;                             // VARIÁVEL PARA GUARDAR O CÓDIGO DE RETORNO DO ENVIO
  HTTPClient http;                              // DECLARAÇÃO DE OBJETO HTTPClient
  http.begin(urlMensagemFogo);                  // RECEBE A URL CRIADA NO INÍCIO DO PROGRAMA
  respostaCode = http.GET();                    // RECEBE O CÓDIGO DE RETORNO 
  if(respostaCode == 200){                      // SE O CÓDIGO FOR IGUAL A 200 A MENSAGEM FOI ENVIADA
    Serial.print("\nMensagem enviada!");
  }
  else{                                         // SE O CÓDIGO RECEBIDO FOR DIFERENTE DE 200 INFORMA ERRO E O RESPECTIVO CÓDIGO  
    Serial.print("\nErro ao enviar mensagem: ");
    Serial.print(respostaCode);
  }
  http.end();
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------FUNÇÃO PARA ENVIAR MENSAGEM BOTÃO DE AJUDA ESTEJA ATIVADO----------------------------------------------------------
void enviaMensagemAjuda(){
                 
  int respostaCode;                            // VARIÁVEL PARA GUARDAR O CÓDIGO DE RETORNO DO ENVIO
  HTTPClient http;                             // DECLARAÇÃO DE OBJETO HTTPClient
  http.begin(urlMensagemAjuda);                // RECEBE A URL CRIADA NO INÍCIO DO PROGRAMA
  respostaCode = http.GET();                   // RECEBE O CÓDIGO DE RETORNO 
  if(respostaCode == 200){                     // SE O CÓDIGO FOR IGUAL A 200 A MENSAGEM FOI ENVIADA
    Serial.print("\nMensagem enviada!");
  }
  else{                                         // SE O CÓDIGO RECEBIDO FOR DIFERENTE DE 200 INFORMA ERRO E O RESPECTIVO CÓDIGO  
    Serial.print("\nErro ao enviar mensagem: ");
    Serial.print(respostaCode);
  }
  http.end();
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------FUNÇÃO QUE MONITORA O SENSOR MQ2 E GERENCIA AÇÕES---------------------------------------------------------------
void monitoraMq2(){

  // VERIFICA SE O SENSOR FOI ATIVADO E SE A FLAG DE CONTROLE ESTÁ COM VALOR FALSO
  if(digitalRead(entradaMq2) == LOW && mq2Acionado == false ){

  digitalWrite(saidaFogoPtencia, HIGH); // LIGA A SAÍDA DE POTÊNCIA EM CASO DE FOGO
  digitalWrite(saidaBuzzerLed, HIGH);   // LIGA O LED E BUZZER EM CASO DE FOGO

  // ENVIA A MENSAGEM DE ALERTA
  enviaMensagemFogo(); 

  // MUDA ESTADO DA VARIÁVEL mq2Acionado PARA QUE O SENSOR SÓ POSSAR SER LIDO DE ACORDO COM O TEMPO DEFINIDO PELA VARIÁVEL tempoParaEnvio
  mq2Acionado = true;
  tempoAnteriorMq2  = millis();
   
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------FUNÇÃO QUE MONITORA O BOTÃO DE AJUDA E GERENCIA AÇÕES----------------------------------------------------------
void monitoraBtnAjuda(){

  // VERIFICA SE O BOTÃO DE AJUDA FOI PRESSIONADO E SE A FLAG DE CONTROLE ESTÁ COM VALOR FALSO
  if(digitalRead(entradaBtnAjuda) == HIGH && btnAjudaAcionado == false){

  digitalWrite(saidaAjudaPotencia, HIGH);  // LIGA A SAÍDA DE POTÊNCIA EM CASO DE PEDIDO DE AJUDA

  // ENVIA A MENSAGEM DE ALERTA
  enviaMensagemAjuda();

  // MUDA ESTADO DA VARIÁVEL btnAjudaAcionado PARA QUE O BOTÃO SÓ POSSAR SER LIDO DE ACORDO COM O TEMPO DEFINIDO PELA VARIÁVEL tempoParaEnvio
  btnAjudaAcionado       = true;
  tempoAnteriorBtnAjuda  = millis();
               
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------



//---------------------------------------FUNÇÃO PARA EFETUAR A LEITURA DO NTC E CONVERTER PARA GRAUS CELSIUS------------------------------------------------------------
void leituraDoNTC(){

  if(ntcFoiLido == false){

  //VARIÁVEIS INTERNAS DA FUNÇÃO
  float Vout, Rt  = 0;             // RESPECTIVAMENTE (TENSÃO QUE CHEGA NO ESP32 APÓS DIVISOR DE TENSÃO ENTRE NTC E R1) E (RESISTÊNCIA DO NTC DE ACORDO A TEMPERATURA ATUAL)
  float T, adc    = 0;             // RESPECTIVAMENTE (TEMPERATURA EM KELVIN) E (VALOR ANALÓGICO LIDO PELO ESP32)
 
    adc = analogRead(entradaNTC);  // VARIÁVEL QUE RECEBE A LEITURA DO NTC
 
  //CALCULOS PARA CONVERSÃO DA LEITURA RECEBIDA PELO ESP32 EM TEMPERATURA EM °C, UTILIZANDO COMO BASE A Steinhart-Hart
  Vout = (adc * Vs)/adcMax;
  Rt = (R1 * Vout) / (Vs - Vout);
  T = 1/(1/To + log(Rt/Ro)/Beta);
  Tc = T - 273.15;

  //IMPRIME NA SERIAL O VALOR EM GRAUS CELSIUS LIDO
  Serial.print("\n Temperatura lida: ");
  Serial.print(Tc);

  // MUDA ESTADO DA VARIÁVEL ntcFoiLido PARA QUE SÓ POSSAR SER LIDA A TEMPERATURA DE ACORDO COM O TEMPO DEFINIDO PELA VARIÁVEL tempoParaLeituraNTC
  ntcFoiLido = true;
  tempoAnteriorNTC = millis();

  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------



//-------------------------------------------------FUNÇÃO PARA MONITORAR A TEMPERATURA LIDO COM NTC----------------------------------------------------------------------
void monitoraNTC(){

  // VERIFICA SE A FLAG ntcMonitorado QUE CONTROLA A CHAMADA DA FUNÇÃO monitoraNTC() ESTÁ COM VALOR FALSO
  if(ntcMonitorado == false){

  // VERIFICA SE A TEMPERATURA LIDA COM NTC É IGUAL OU SEPERIOR AO VALOR DEFINIDO PELA VARIÁVEL Tc (TEMPERATURA ALTA INDICANDO FOGO)
    if(Tc > temperaturaAlertaNTC){

      digitalWrite(saidaFogoPtencia, HIGH);  // LIGA A SAÍDA DE POTÊNCIA EM CASO DE FOGO
      digitalWrite(saidaBuzzerLed, HIGH);    // LIGA O LED E BUZZER EM CASO DE FOGO

  // ENVIA A MENSAGEM DE ALERTA
      enviaMensagemFogo(); 

  // MUDA ESTADO DA VARIÁVEL ntcMonitorado PARA QUE A FUNÇÃO monitoraNTC() SÓ POSSA SER CHAMADA DE ACORDO COM O TEMPO DA VARIÁVEL tempoParaEnvio
      ntcMonitorado = true;
      tempoAnteriorMonitoraNTC = millis();

    }
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------


