/*
  MB4PT100.cpp - v1.00 - 12/10/2015
  - Version inicial
  
  Sketch para el módulo acondicionador de sensores PT1000/100
  Copyright (c) 2014 Raimundo Alfonso
  Ray Ingeniería Electrónica, S.L.
  
  Este sketch está basado en software libre. Tu puedes redistribuir
  y/o modificarlo bajo los terminos de licencia GNU.

  Esta biblioteca se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los terminos de licencia GNU para más detalles.
  
  * CARACTERISTICAS
  - Medida de temperatura 0.1ºC resolución
  - Rango desde -150ºC hasta +850ºC
  - Medida de PT100 o PT1000 a 2, 3 y 4 hilos.
  - 4 Canales
  - Sensor de temperatura en PCB
  - 6 interruptores dipswitch para direccionamiento modbus
  - Bus de comunicaciones RS485 con detección automática de dirección
  - Amplio rango de alimentación de 6.5 a 30VDC
  - Regulador conmutado de alta eficiencia
  
  
  * MAPA MODBUS
    MODO R: FUNCION 3 - READ BLOCK HOLDING REGISTERS
    MODO W: FUNCION 6 - WRITE SINGLE HOLDING REGISTER
    
  DIRECCION   TIPO    MODO  FORMATO    MAXIMO      MINIMO    UNIDADES    DESCRIPCION
  ---------------------------------------------------------------------------------------------------------
  0x0000      int     R     0000.0    +0850.0     -0150.0    ºC          TEMPERATURA CANAL 1
  0x0001      int     R     0000.0    +0850.0     -0150.0    ºC          TEMPERATURA CANAL 2
  0x0002      int     R     0000.0    +0850.0     -0150.0    ºC          TEMPERATURA CANAL 3
  0x0003      int     R     0000.0    +0850.0     -0150.0    ºC          TEMPERATURA CANAL 4
  0x0004      uint    R     00000     00001        00000     ---         FUERA DE RANGO CANAL 1
  0x0005      uint    R     00000     00001        00000     ---         FUERA DE RANGO CANAL 2
  0x0006      uint    R     00000     00001        00000     ---         FUERA DE RANGO CANAL 3
  0x0007      uint    R     00000     00001        00000     ---         FUERA DE RANGO CANAL 4
  0x0008      int     R     0000.0    +00155.0    -00055.0   ºC          TEMPERATURA EN PCB
  0x0009      uint    R     00000      00063       00000     ---         ESTADO DEL DIPSWITCH
  0x000A      uint    R     0000.0     65535       00000     Ohm         RESISTENCIA CANAL 1
  0x000B      uint    R     0000.0     65535       00000     Ohm         RESISTENCIA CANAL 2
  0x000C      uint    R     0000.0     65535       00000     Ohm         RESISTENCIA CANAL 3
  0x000D      uint    R     0000.0     65535       00000     Ohm         RESISTENCIA CANAL 4  
*/

#include <ModbusSlave.h>
#include <MCP3551.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h> 


#define DIPSW1	10   // Dirección modbus 0
#define DIPSW2	9    // Dirección modbus 1
#define DIPSW3	8    // Dirección modbus 2
#define DIPSW4	7    // Dirección modbus 3
#define DIPSW5	6    // Paridad none/par
#define DIPSW6	5    // Velocidad 9600/19200
#define CS1     14   // CS AD1
#define CS2     15   // CS AD2
#define CS3     16   // CS AD3
#define CS4     17   // CS AD4
#define PT100SEL  3  // PT100 selector
#define DS18B20 4    // Sensor de tempertura en placa

#define MAX_BUFFER_RX  15

// Crea variables globales... 
float temperature[4];
unsigned long currentMillis  = 0;
unsigned long previousMillis = 0;
byte cnt = 0;
char buffer_rx[MAX_BUFFER_RX];
unsigned int res_min = 39;
unsigned int res_max = 400;

float rtd[4];
float calRA[4] = {6800,6800,6800,6800};
int RZero = 100;


// Mapa de registros modbus
enum {        
        MB_TEMPERATURA1,  // Temperatura canal 1
        MB_TEMPERATURA2,  // Temperatura canal 2
        MB_TEMPERATURA3,  // Temperatura canal 3
        MB_TEMPERATURA4,  // Temperatura canal 4
        MB_OUT_R1,        // Fuera de rango canal 1
        MB_OUT_R2,        // Fuera de rango canal 2
        MB_OUT_R3,        // Fuera de rango canal 3
        MB_OUT_R4,        // Fuera de rango canal 4
        MB_TEMPERATURA,   // Temperatura en placa
        MB_DIP,           // Estado dipswitch
        MB_RES1,          // Resistencia canal 1
        MB_RES2,          // Resistencia canal 2
        MB_RES3,          // Resistencia canal 3
        MB_RES4,          // Resistencia cabal 4
        MB_REGS	 	  // Numero total de registros
};
int regs[MB_REGS];	

// Crea la clase para el modbus...
ModbusSlave modbus;

// Crea la clase para lectura de rtd
MCP3551 rtd1(CS1);
MCP3551 rtd2(CS2);
MCP3551 rtd3(CS3);
MCP3551 rtd4(CS4);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(DS18B20);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

void setup()  { 
  wdt_disable();
  int velocidad;
  char paridad;

  // Configura puertos de Arduino  
  pinMode(DIPSW1,INPUT);
  pinMode(DIPSW2,INPUT);	
  pinMode(DIPSW3,INPUT);	
  pinMode(DIPSW4,INPUT);	
  pinMode(DIPSW5,INPUT);	
  pinMode(DIPSW6,INPUT);	
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);
  pinMode(CS4, OUTPUT);  
  pinMode(PT100SEL, INPUT);

  digitalWrite(CS1,HIGH);
  digitalWrite(CS2,HIGH);
  digitalWrite(CS3,HIGH);
  digitalWrite(CS4,HIGH);  
  
  
  // configura modbus...
  modbus.direccion = leeDIPSW();
  modbus.config(9600,'n');
  
  // Activa WDT cada 4 segundos...   
  wdt_enable(WDTO_4S); 
  
  // Inicializa sensor DS18B20
  sensors.begin();

  // Configura puerto SPI...
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32); //configurate SPI
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  
  // Comprueba si estamos en modo PT100 o PT1000
  if(digitalRead(PT100SEL) == false){  // Si el jumper está puesto...
    RZero = 1000; 
    res_min *= 10;
    res_max *= 10;
  }else{
    RZero = 100;
  }

  // Lee la primera muestra...
  regs[MB_OUT_R1] = 1;
  regs[MB_OUT_R2] = 1;
  regs[MB_OUT_R3] = 1;
  regs[MB_OUT_R4] = 1;  
  rtd1.getCode();
  rtd2.getCode();
  rtd3.getCode();
  rtd4.getCode();  
  delay(100);
} 



void loop()  { 
  
  // Lee temperatura del sensor DS18B20+
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); 
  sensors.setWaitForConversion(true);
  
  // Lee RTDs...
  bool rtd1Ready = rtd1.getCode();
  bool rtd2Ready = rtd2.getCode();
  bool rtd3Ready = rtd3.getCode();
  bool rtd4Ready = rtd4.getCode();  
  if(rtd1Ready){
    rtd[0] = calRA[0] * (float(rtd1.byteCode) / ( 1048576.0 - float(rtd1.byteCode))); 
    regs[MB_OUT_R1] = 0;
    if(rtd[0] > res_max){
      rtd[0] = res_max;  
      regs[MB_OUT_R1] = 1;      
    }
    if(rtd[0] < res_min){
      rtd[0] = res_min; 
      regs[MB_OUT_R1] = 1;      
    }
    regs[MB_RES1] = int(rtd[0] * 10.0);
    rtd[0] = (rtd[0] / RZero) - 1;
    temperature[0] = (rtd[0] * (255.8723 + rtd[0] * (9.6 + rtd[0] * 0.878)));
    regs[MB_TEMPERATURA1] = int(temperature[0] * 10);     
  }
  if(rtd2Ready){
    rtd[1] = calRA[1] * (float(rtd2.byteCode) / ( 1048576.0 - float(rtd2.byteCode)));   
    regs[MB_OUT_R2] = 0;
    if(rtd[1] > res_max){
      rtd[1] = res_max;  
      regs[MB_OUT_R2] = 1;      
    }
    if(rtd[1] < res_min){
      rtd[1] = res_min; 
      regs[MB_OUT_R2] = 1;      
    }    
    regs[MB_RES2] = int(rtd[1] * 10.0);
    rtd[1] = (rtd[1] / RZero) - 1;
    temperature[1] = (rtd[1] * (255.8723 + rtd[1] * (9.6 + rtd[1] * 0.878)));
    regs[MB_TEMPERATURA2] = int(temperature[1] * 10);         
  }
  if(rtd3Ready){
    rtd[2] = calRA[2] * (float(rtd3.byteCode) / ( 1048576.0 - float(rtd3.byteCode)));    
    regs[MB_OUT_R3] = 0;
    if(rtd[2] > res_max){
      rtd[2] = res_max;  
      regs[MB_OUT_R3] = 1;      
    }
    if(rtd[2] < res_min){
      rtd[2] = res_min; 
      regs[MB_OUT_R3] = 1;      
    }    
    regs[MB_RES3] = int(rtd[2] * 10.0);
    rtd[2] = (rtd[2] / RZero) - 1;
    temperature[2] = (rtd[2] * (255.8723 + rtd[2] * (9.6 + rtd[2] * 0.878)));
    regs[MB_TEMPERATURA3] = int(temperature[2] * 10);         
  }
  if(rtd4Ready){
    rtd[3] = calRA[3] * (float(rtd4.byteCode) / ( 1048576.0 - float(rtd4.byteCode)));    
    regs[MB_OUT_R4] = 0;
    if(rtd[3] > res_max){
      rtd[3] = res_max;  
      regs[MB_OUT_R4] = 1;      
    }
    if(rtd[3] < res_min){
      rtd[3] = res_min; 
      regs[MB_OUT_R4] = 1;      
    }    
    regs[MB_RES4] = int(rtd[3] * 10.0);
    rtd[3] = (rtd[3] / RZero) - 1;
    temperature[3] = (rtd[3] * (255.8723 + rtd[3] * (9.6 + rtd[3] * 0.878)));
    regs[MB_TEMPERATURA4] = int(temperature[3] * 10);         
  }

  regs[MB_TEMPERATURA] = int(sensors.getTempCByIndex(0) * 10);

  // Asigna valores a la tabla modbus...
  regs[MB_DIP] = leeDIPSW();

  modbus.actualiza(regs,MB_REGS);  
  wdt_reset();
}

// Rutina de espera que atiende la tarea modbus...
void delay_modbus(int t){
  int n,tt;
  tt = t/10;
  
  for(n=0;n<=tt;n++){
    modbus.actualiza(regs,MB_REGS);
    delay(10);
  }  
}

// Rutina para leer el dipswitch
byte leeDIPSW(void){
  byte a0,a1,a2,a3,a4,a5;
  
  // Lee dipswitch...
  a0 = !digitalRead(DIPSW1);  
  a1 = !digitalRead(DIPSW2);
  a2 = !digitalRead(DIPSW3);
  a3 = !digitalRead(DIPSW4);
  a4 = !digitalRead(DIPSW5);  
  a5 = !digitalRead(DIPSW6);  

  // Calcula dirección...
  return(a0 + a1*2 + a2*4 + a3*8 + a4*16 + a5*32);
}

