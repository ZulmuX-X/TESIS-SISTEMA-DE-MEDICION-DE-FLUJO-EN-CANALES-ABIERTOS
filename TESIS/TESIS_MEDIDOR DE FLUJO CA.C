//*********************************************************************
// PROYECTO:  SISTEMA ELECTRONICO PORTÁTIL PARA SIMPLIFICAR LA MEDICIÓN 
//            DE FLUJO DE AGUA EN CANAL ABIERTO.
// MEMORIA UTILIZADA:  ROM:73%      RAM:42%
// COMPILADOR: CCS  PCW COMPILER V 4.104   http://www.ccsinfo.com/
// PROGRAMADOR DEPURADOR Z2 DE PICTRONICO 
// http://www.pictronico.com/
// CHIP PIC18F4550 I/P  DIP
// CRISTAL DE 20 MHZ.
//*********************************************************************


#include <18F4550.h>
#include <math.h>

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES HSPLL                    //High Speed Crystal/Resonator with PLL enabled
#FUSES NOPROTECT                //Code not protected from reading
#FUSES BROWNOUT                 //Reset when brownout detected
#FUSES BORV28                   //Brownout reset at 2.8V
#FUSES PUT                      //Power Up Timer
#FUSES NOCPD                    //No EE protection
#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOWRT                    //Program memory not write protected
#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES NOIESO                   //Internal External Switch Over mode disabled
#FUSES NOFCMEN                  //Fail-safe clock monitor disabled
#FUSES NOPBADEN                 //PORTB pins are configured as digital I/O on RESET
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOEBTRB                  //Boot block not protected from table reads
#FUSES NOCPB                    //No Boot Block code protection
#FUSES NOMCLR                   //Master Clear pin used for I/O
#FUSES NOLPT1OSC                //Timer1 configured for higher power operation
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES PLL5                     //Divide By 5(20MHz oscillator input)
#FUSES CPUDIV1                  //No System Clock Postscaler
#FUSES USBDIV                   //USB clock source comes from PLL divide by 2
#FUSES VREGEN                   //USB voltage regulator enabled
#FUSES NOICPRT  
#use delay(clock=48000000)


#DEFINE USB_HID_DEVICE TRUE 
#define USB_EP1_TX_ENABLE USB_ENABLE_INTERRUPT 
#define USB_EP1_TX_SIZE 48 //este se cambia en desc_hid.h y usb.c

#define USB_EP1_RX_ENABLE USB_ENABLE_INTERRUPT
#define USB_EP1_RX_SIZE 48
#define usb_con_sense_pin pin_a5
#define INICIO PIN_A1 // Defino el Pin DE INICIO
#define VBAJO PIN_A3 // Defino el Pin DE DETECCION BATERIA BAJA
#define LED PIN_A2 // Defino el Pin del BUZZER
#define FLASH Output_Toggle(LED) // Defino la funcion Flash 
////////////////////////////////////////////////////////////////////////////////////
#include <pic18_usb.h>
#include "usb_desc_hid.h"
#include <usb.c> 
#define use_portD_lcd TRUE
#include <LCDD.c>

#use standard_io(a)
#use standard_io(b)
#use standard_io(c)
#use standard_io(e)
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7, bits=8, parity=N)

float const uSxTick = 0.1666667; // Microsegundos por Tick de TMR1 a 20 Mhz

////////////////////////////////////////////////////////////////////////////////////
// VARIABLES GLOBALES
///////////////////////////////////////////////////////////////////////////////////
int8 numFlancoQueLlega=0; // Número de Flanco que llega
int8 j;
int8 k=0;
int med=0;

int8 indice=0;
int8 solo=0;
float llave=0;
float suma=0;
float suma2=0;
int8 via=0;
int8 muestra=1;//contador de muestras
float mu[37];//muestras
float var[11];// registro ultimos valores
float fprom=0;
float rpm=0;
float freal=0;
int1 flagToggleFlanco=0; // Flag para cambiar de flanco
float t1=0,t2=0,t3=0; // Variables para guardar estados de ...
float tiempoprueba=0, vel=0;
float tiempoprueba2=0;
 // Variables para guardar estados de ...
float tth1=0,ttl1=0; // Timers y pulsos.
float sth1=0,stl1=0; // Timers y pulsos.
float ttt=0; // Tiempo total de bajo
float m,b,r,s;
int16 nTimer1Overflow=0; // Contador de Interrupciones
float nTimer1Overflow2=0; // Contador de Interrupciones
int16 sobreflujo=0;
int16 sobreflujoant=0;
// contador de sobreflujos del timer para frecuencias bajss T > 13.10 ms

float f=0.00; // Para hacer las restas oportunas en uS
int1 flagHayDatos=0; // Flag para indicar que ya hay datos de ..
// dos flancos (de subida y bajada)
int1 flagHayTransmitir=0; // Flag para indicar que hay datos para ...
int1 mensaje=0;


//char out_dato[48];
int8 a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32;
int8 a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48;


char c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16;//para ver si la letra se despliega 
float b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12,b13,b14,b15,b16;
float b17,b18,b19,b20,b21,b22,b23,b24,b25,b26,b27,b28,b29,b30,b31,b32;
char in_dato[48];



#int_timer1
void interrupt_service_rutine_timer1(void) {
    ++nTimer1Overflow;//contador de sobreflujos
     nTimer1Overflow2=nTimer1Overflow2+1;//contador de sobreflujos
tiempoprueba=(nTimer1Overflow2 *65536) + get_timer1(); 
tiempoprueba2=1.046*uSxTick *(tiempoprueba/1000000);//modifique el tiempo de prueba de 2000000 a 1000000 17nov2010      
sobreflujo=nTimer1Overflow2-sobreflujoant;//sobreflujo ant es cuenta memoria
if((sobreflujo>60)&&(llave==1)&&(f<4)&&(via<2)){// a lo mejor con 50 ya lo hace continuo
lcd_gotoxy(1,2); //COL Y RENGLON
printf(lcd_putc,"TMPO PRUEBA:%4.0f \n",tiempoprueba2);// imprimo tiempo de prueba
sobreflujoant=nTimer1Overflow2;  

                  }

                                             }

////////////////////////////////////////////////////////////////////////////////////
// Interrupción por Externa por Cambio de Flanco en RB0
// SE MANEJA OVERFLOW EN TIMER1 
// En programa ocurre error por mitad de frecuencia
// ademas debe estar regulado en filtro histeresis
////////////////////////////////////////////////////////////////////////////////////
#int_ext
void handle_ext_int(){//int ext
++numFlancoQueLlega; // Cuento flanco que nos llega

if(flagToggleFlanco==0){ // He recibido Flanco de Subida
    if(numFlancoQueLlega==1){
         set_timer1(0); // Reinicio TMR1
         nTimer1Overflow=0;//sobreflujo inicializado
         t1=get_timer1(); // Guardo en t1 el valor de TMR1 al primer Flanco de Subida
                            }

              if(numFlancoQueLlega==3){
                                      t3= (nTimer1Overflow *65536) + get_timer1(); // Guardo en t1 el valor de TMR1 al primer Flanco de Subida

                            if(flagHayDatos==0){ // Si los datos anteriores han sido procesados ...
                                                flagHayDatos=1; // Indico que ya hay nuevos datos de flancos para calcular
                                               }
                                       }
                         ext_int_edge(0,H_TO_L); // Configuro para capturar siguiente flanco de Bajada
                         flagToggleFlanco=1; // Indico que el siguiente flanco será de Bajada
                      }else { // He recibido Flanco de Bajada
                            t2= (nTimer1Overflow *65536) + get_timer1(); // Guardo en t2 el valor de TMR1 al Flanco de Bajada
                            ext_int_edge(0,L_TO_H); // Configuro para capturar siguiente flanco de subida
                            flagToggleFlanco=0; // Indico que el siguiente flanco será de Subida                    
                            }
                      if(llave>0){
                                 FLASH; // Reproduzco la entrada mediante un LEd en E0;
                                 }           
                      if(numFlancoQueLlega==3){
                                              numFlancoQueLlega=0;//INICIALIZO Y SALGO
                                              }
                    }//int ext
//********************************************************************
//********************************************************************




//***************************************************************
//***************************************************************
//se debe de ver los valores almacenados en 2 renglones cada vez
// que se presiona el boton 
void rutina3()
{
do{

                         
                             if(k==0){
                                lcd_gotoxy(1,1);
                                printf(lcd_putc,"                ");//imprimo rpm
                                lcd_gotoxy(1,1);
                                printf(lcd_putc,"DATO[1]= %3.3f \n",var[1]);//imprimo rpm
                                lcd_gotoxy(1,2);
                                printf(lcd_putc,"DATO[2]= %3.3f \n",var[2]);//imprimo rpm
                                delay_ms(2000);
                                k=1;
                                      }

                                       if( input(INICIO)&&(k==1) ){//inicio SIG DATOS
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"                ");//imprimo rpm
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"DATO[3]= %3.3f \n",var[3]);//imprimo rpm
                                                          lcd_gotoxy(1,2);
                                                          printf(lcd_putc,"DATO[4]= %3.3f \n",var[4]);//imprimo rpm  
                                                          delay_ms(2000);
                                                          k=2;
                                                          }//inicio SIG DATOS
                                       if( input(INICIO)&&(k==2) ){//inicio SIG DATOS
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"                ");//imprimo rpm
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"DATO[5]= %3.3f \n",var[5]);//imprimo rpm
                                                          lcd_gotoxy(1,2);
                                                          printf(lcd_putc,"DATO[6]= %3.3f \n",var[6]);//imprimo rpm  
                                                          delay_ms(2000);
                                                          k=3;
                                                          }//inicio SIG DATOS
                                       if( input(INICIO)&&(k==3) ){//inicio SIG DATOS
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"                ");//imprimo rpm
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"DATO[7]= %3.3f \n",var[7]);//imprimo rpm
                                                          lcd_gotoxy(1,2);
                                                          printf(lcd_putc,"DATO[8]= %3.3f \n",var[8]);//imprimo rpm
                                                          delay_ms(2000);
                                                          k=4;
                                                          }//inicio SIG DATOS
                                                          
                                       if( input(INICIO)&&(k==4) ){//inicio SIG DATOS
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"                ");//imprimo rpm
                                                          lcd_gotoxy(1,1);
                                                          printf(lcd_putc,"DATO[9]= %3.3f \n",var[9]);//imprimo rpm
                                                          lcd_gotoxy(1,2);
                                                          printf(lcd_putc,"DATO[10]= %3.3f \n",var[10]);//imprimo rpm 
                                                          delay_ms(2000);
                                                          k=5;
                                                          }//inicio SIG DATOS
                                                        
                                                       if( input(INICIO)&&(k==5) ){//inicio SIG DATOS
                                                          printf(lcd_putc,"RETORNO MEDICION");//imprimo rpm
                                                          delay_ms(2000);
                                                          via=0;
                                                          med=1;
                                                          k=0;
                                                          }//inicio SIG DATOS                                      
                                    
 //rutina para ver registro en medicion y ver datos presionando el boton
 //puede abortar si se presiona el boton dos veces rapido otra vez
}while((llave>0)&&(via==2));
}//rutina3


void rutina2()
{


                                    
do{
if(flagHayDatos==1){ // Detecto que ya hay datos de flancos ...
       if((t3>t2)&&(t2>t1)){ // Compruebo que estoy en la misma vuelta de TMR1
                            tth1 = t2 - t1; // Tiempo alto  ticks
                            ttl1 = t3 - t2; // Tiempo bajo  ticks                            
                            sth1 = uSxTick * tth1; //en us
                            stl1 = uSxTick * ttl1; //en us                                                 
                            ttt=((sth1+stl1)/1000000);;//considerando 50% talto
                            f = 1 /(ttt); // Calculo la Frecuencia
                            //correccion de frecuencia
                            
                            flagHayTransmitir=1; // Indico que tengo nuevo valor para transmitir lo reacomode 10

                            }//if
                      flagHayDatos=0; // Indico que ya han sido procesados los datos.
                     }//flagHayDatos==1

        
                   if(flagHayTransmitir==1){ // Si hay algo pendiente de transmitir ...
                         
//SOLO SE ENTRA SI HAY DATOS DE FRECUENCIA                          
if((llave>0)&&(f<12)){//llave y f
if(solo==0){
lcd_gotoxy(1,1);
printf(lcd_putc,"                  ");
lcd_gotoxy(1,2);
printf(lcd_putc,"                  ");

solo=1;
             }//solo limpia una vez
              

if(f<0){ f=freal;}

if(muestra==1){//solo si  muestra es uno
fprom=f;//solo la prinera toma el valor de faterior o freal
               }//solo si  muestra es uno
               
if( (f>0)&&(muestra>0) ){//si f>0 y valor no esta desviado
mu[muestra]=f;
suma=suma+ mu[muestra];
fprom=suma/(muestra);

                        }//si f>0
                        
lcd_gotoxy(11,1); //COL  RENGLON
printf(lcd_putc,"V:%3.2f \n",vel); //imprimno vel
printf("V:%3.2f \n",vel);



if( (muestra>=2)&&(freal<=0.5) ){//contador de muestra
//VARIOS RANGOS DE VARIACION
//COMPARA SI LA LECTURA ACTUAL ES MAYOR QUE EL PROMEDIO (ABS) LEC ACT=PROMEDIO
for(j=1; j<= muestra; j++){// AQUI RUTINA DE COMP
if( abs(fprom-mu[j])>1 ){
                  mu[j]=fprom;
                        }
suma2=suma2+mu[j];
                         }// AQUI RUTINA DE COMP
freal= suma2/muestra; //promedio descartando lecturas falsas 


//*******************SE IMPRIME****************************************
//*********************************************************************
rpm=freal*60 ;
vel=m*rpm+b;//calculo de la velocidad v= m*rpm+b     //general
lcd_gotoxy(1,1); //COL  RENGLON
printf(lcd_putc,"RPM:%3.2f",rpm);//imprimo rpm
printf("RPM:%3.2f",rpm);
//*******************SE IMPRIME****************************************
//*********************************************************************             
muestra=0; // SE REINICIA EL CONTADOR DE MUESTRAS
fprom=0;
suma=0;
suma2=0;
delay_ms(300);
                             }//contador de muestra
                             

if( (muestra>=3)&& (freal>=0.5)&& (freal<=12) ){//contador de muestra
//VARIOS RANGOS DE VARIACION 
//COMPARA SI LA LECTURA ACTUAL ES MAYOR QUE EL PROMEDIO (ABS) LEC ACT=PROMEDIO
for(j=1; j<= muestra; j++){// AQUI RUTINA DE COMP
if( abs(fprom-mu[j])>1 ){
                  mu[j]=fprom;
                        }
suma2=suma2+mu[j];
                         }// AQUI RUTINA DE COMP
freal= suma2/muestra; //promedio descartando lecturas falsas 




//*******************SE IMPRIME****************************************
//*********************************************************************
rpm=freal*60;

vel=m*rpm+b;//calculo de la velocidad v= m*rpm+b     //general
lcd_gotoxy(1,1); //COL  RENGLON
printf(lcd_putc,"RPM:%3.2f",rpm);//imprimo rpm
//*******************SE IMPRIME****************************************
//*********************************************************************

//************************************************************************
muestra=0; // SE REINICIA EL CONTADOR DE MUESTRAS
fprom=0;
suma=0;
suma2=0;
delay_ms(300);
//************************************************************************
//*******************SE IMPRIME****************************************
//*********************************************************************             
                             }//contador de muestra
                             


                                    
//**********************                                   
 if(indice<=10){
var[indice]=rpm;
             }  
 

//***********************

//**********************************************************************
//**********************************************************************
 //rutina para ver registro en medicion y ver datos presionando el boton
 //puede abortar si se presiona el boton dos veces rapido otra vez
                         
                           if( input(INICIO) ){//inicio3  reset
                                lcd_gotoxy(1,1);
                                printf(lcd_putc,"                ");//imprimo rpm
                                lcd_gotoxy(1,1);
                                printf(lcd_putc,"VER REGISTRO");//imprimo rpm
                                via=2;//ver registro
                                delay_ms(2000);
                                               } //inicio3  
 //rutina para ver registro en medicion y ver datos presionando el boton
 //puede abortar si se presiona el boton dos veces rapido otra vez

                                     
//GRABA LOS VALORES EN VAR PARA DESPLEGARLOS


if(indice==10){
var[1]=var[2];
var[2]=var[3];  // PARA SABER SI GUARDA LOS DATOS PONGO CONST Y VEO SI LLEGAN
var[3]=var[4];
var[4]=var[5];
var[5]=var[6];
var[6]=var[7];
var[7]=var[8];
var[8]=var[9];
var[9]=var[10];
indice=9;
              }
//****************************************              
lcd_gotoxy(1,2); //COL Y RENGLON
if((f>=2)&&(via<2)){
printf(lcd_putc,"TMPO PRUEBA:%4.0f\n",tiempoprueba2);
       }          
indice=indice+1;                       
muestra=muestra+1;
//                    }//descartar medicion 
//****************************************

////FIN  DE RUTINA DE COMPARACION DE MUESTRAS                   
//AQUI TERMINA LA LLAVE PRINCIPAL DE LA RUTINA 2                                    
                    }   //llave y f

                  flagHayTransmitir=0; // Indico que ya he transmitido lo pendiente.
                                            }//flag hay datos transmitir

}while((llave>0)&&(via==1));
                   
}//rutina 2


//***************************************************************************
//***********FIN DE RUTINA 2  ***********************************************
//***************************************************************************


//***************************************************************************
//************INICIO DE RUTINA 1 ********************************************
//***************************************************************************

void rutina1()
{
WHILE(1){//1 while
   output_low(pin_b7);
   output_low(pin_b6);
   output_low(pin_b3);
   output_low(pin_b2);
   usb_task();
 
    output_low(pin_E1);
 
  
 if( usb_attached()){  
  if (usb_enumerated()){

  output_high(pin_E1);
  output_low(pin_E0);

  if (usb_kbhit(1)) {
            usb_get_packet(1, in_dato,49);
            
            
           
            write_eeprom(0,in_dato[0]);// escribe un cero
           
            write_eeprom(1,in_dato[1]);
           
            write_eeprom(2,in_dato[2]);
           
            write_eeprom(3,in_dato[3]);
           
            write_eeprom(4,in_dato[4]);// escribe 4 valores deben ser 20
            
            write_eeprom(5,in_dato[5]);
           
            write_eeprom(6,in_dato[6]); 
           
            write_eeprom(7,in_dato[7]);  
           
            write_eeprom(8,in_dato[8]);// escribe 4 valores deben ser 20        
           
            write_eeprom(9,in_dato[9]);
            
            write_eeprom(10,in_dato[10]);
           
            write_eeprom(11,in_dato[11]);
           
            write_eeprom(12,in_dato[12]);// escribe 4 valores deben ser 20
           
            write_eeprom(13,in_dato[13]);
           
            write_eeprom(14,in_dato[14]);
           
            write_eeprom(15,in_dato[15]);                 
    
           
            write_eeprom(16,in_dato[16]);
            
            write_eeprom(17,in_dato[17]);// 
           
            write_eeprom(18,in_dato[18]);
           
            write_eeprom(19,in_dato[19]);
           
            write_eeprom(20,in_dato[20]); 
           
            write_eeprom(21,in_dato[21]);
            
            write_eeprom(22,in_dato[22]);// 
            
            write_eeprom(23,in_dato[23]);
            
            
            write_eeprom(24,in_dato[24]);
           
            write_eeprom(25,in_dato[25]); 
            
            write_eeprom(26,in_dato[26]);
          
            write_eeprom(27,in_dato[27]);// 
           
            write_eeprom(28,in_dato[28]);
          
            write_eeprom(29,in_dato[29]);
           
            write_eeprom(30,in_dato[30]); 
          
            write_eeprom(31,in_dato[31]);
            
            
            write_eeprom(32,in_dato[32]);// 
           
            write_eeprom(33,in_dato[33]);
          
            write_eeprom(34,in_dato[34]);
           
            write_eeprom(35,in_dato[35]);
            
            write_eeprom(36,in_dato[36]);
            
            write_eeprom(37,in_dato[37]);// 
           
            write_eeprom(38,in_dato[38]);
           
            write_eeprom(39,in_dato[39]);
            
            
            write_eeprom(40,in_dato[40]);
           
            write_eeprom(41,in_dato[41]);
            
            write_eeprom(42,in_dato[42]);// 
            
            write_eeprom(43,in_dato[43]);
           
            write_eeprom(44,in_dato[44]);
           
            write_eeprom(45,in_dato[45]); 
           
            write_eeprom(46,in_dato[46]);
           
            write_eeprom(47,in_dato[47]);
            
         //   usb_put_packet(1,out_dato,48,USB_DTS_TOGGLE); 
            delay_ms(5);
  output_high(pin_E0);
  

  delay_ms(2000);
  }
                 }
                 
                 }

   
   } //1 while 
}//rutina 1
//***************************************************************************
//********************FIN DE RUTINA 1 ***************************************
//***************************************************************************

//****************************************************************************
//****************************************************************************
//AQUI EMPIEZA EL PROGRAMA PRINCIPAL
void main()
{
set_tris_a(0b11111010);
set_tris_b(0b00000001);
set_tris_d(0b00000000);
set_tris_c(0b10011001);
set_tris_e(0);
delay_ms(333);
disable_interrupts(global); // Inicializo el Micro y ...
disable_interrupts(int_timer1); // deshabilitando todo lo no necesario ...
disable_interrupts(int_rda);
disable_interrupts(int_ext);
disable_interrupts(int_ext1);
disable_interrupts(int_ext2);
setup_adc_ports(NO_ANALOGS);
setup_adc(ADC_OFF);
setup_spi(FALSE);
setup_psp(PSP_DISABLED);
setup_counters(RTCC_INTERNAL,RTCC_DIV_2);
setup_timer_0(RTCC_OFF);
setup_timer_1(T1_INTERNAL | T1_DIV_BY_2);// de 2 a 1 es mas lento
setup_timer_2(T2_DISABLED,0,1);// SI SE DEJA 1 ES POR 2.4 Y SI SE DEJA 2 ES POR 1.2
setup_timer_3(T3_DISABLED);
setup_comparator(NC_NC_NC_NC);
setup_vref(FALSE);
port_b_pullups(FALSE);

  
ext_int_edge(0,L_TO_H); // Configuro captura de 1er flanco de subida
flagToggleFlanco = 0; // inicializo el Flag para cambiar de flanco
enable_interrupts(int_ext);
enable_interrupts(int_timer1);
enable_interrupts(global);


output_High(pin_e2);
delay_ms(200);
output_low(pin_E2);
delay_ms(200);
output_High(pin_E2);
delay_ms(200);
output_low(pin_E2);
delay_ms(200);
output_high(pin_E2);

output_low(pin_E1);
output_low(pin_E0);

             
usb_init_cs();

delay_ms(333);// di mas tiempo 333-500ms



lcd_init();//anexe mlm
delay_ms(333);// di mas tiempo 333-500ms
// rutina de despliegue de m y b
//*************************************************************************


 c1 = read_eeprom(0);   //se lee los datos de la eeprom
 c2 = read_eeprom(1);   //se lee los datos de la eeprom
 c3 = read_eeprom(2);   //se lee los datos de la eeprom
 c4 = read_eeprom(3);   //se lee los datos de la eeprom
 c5 = read_eeprom(4);   //se lee los datos de la eeprom
 c6 = read_eeprom(5);   //se lee los datos de la eeprom
 c7 = read_eeprom(6);   //se lee los datos de la eeprom
 c8 = read_eeprom(7);   //se lee los datos de la eeprom
 c9 = read_eeprom(8);   //se lee los datos de la eeprom
 c10 = read_eeprom(9);   //se lee los datos de la eeprom
 c11 = read_eeprom(10);   //se lee los datos de la eeprom
 c12 = read_eeprom(11);   //se lee los datos de la eeprom
 c13 = read_eeprom(12);   //se lee los datos de la eeprom
 c14 = read_eeprom(13);   //se lee los datos de la eeprom
 c15 = read_eeprom(14);   //se lee los datos de la eeprom
 c16 = read_eeprom(15);   //se lee los datos de la eeprom
 
 a17 = read_eeprom(16);   //se lee los datos de la eeprom
 a18 = read_eeprom(17);   //se lee los datos de la eeprom
 a19 = read_eeprom(18);   //se lee los datos de la eeprom
 a20 = read_eeprom(19);   //se lee los datos de la eeprom
 a21 = read_eeprom(20);   //se lee los datos de la eeprom
 a22 = read_eeprom(21);   //se lee los datos de la eeprom
 a23 = read_eeprom(22);   //se lee los datos de la eeprom 
 a24 = read_eeprom(23);   //se lee los datos de la eeprom
 
 a25 = read_eeprom(24);   //se lee los datos de la eeprom
 a26 = read_eeprom(25);   //se lee los datos de la eeprom
 a27 = read_eeprom(26);   //se lee los datos de la eeprom
 a28 = read_eeprom(27);   //se lee los datos de la eeprom
 a29 = read_eeprom(28);   //se lee los datos de la eeprom
 a30 = read_eeprom(29);   //se lee los datos de la eeprom
 a31 = read_eeprom(30);   //se lee los datos de la eeprom
 a32 = read_eeprom(31);   //se lee los datos de la eeprom
 
 a33 = read_eeprom(32);   //se lee los datos de la eeprom
 a34 = read_eeprom(33);   //se lee los datos de la eeprom
 a35 = read_eeprom(34);   //se lee los datos de la eeprom
 a36 = read_eeprom(35);   //se lee los datos de la eeprom
 a37 = read_eeprom(36);   //se lee los datos de la eeprom
 a38 = read_eeprom(37);   //se lee los datos de la eeprom
 a39 = read_eeprom(38);   //se lee los datos de la eeprom 
 a40 = read_eeprom(39);   //se lee los datos de la eeprom
 
 a41 = read_eeprom(40);   //se lee los datos de la eeprom
 a42 = read_eeprom(41);   //se lee los datos de la eeprom
 a43 = read_eeprom(42);   //se lee los datos de la eeprom
 a44 = read_eeprom(43);   //se lee los datos de la eeprom
 a45 = read_eeprom(44);   //se lee los datos de la eeprom
 a46 = read_eeprom(45);   //se lee los datos de la eeprom
 a47 = read_eeprom(46);   //se lee los datos de la eeprom
 a48 = read_eeprom(47);   //se lee los datos de la eeprom
 
 
 
 //paso de parametros
 b1=a17;  //NO SERIE
 b2=a18; 
 b3=a19;  
 b4=a20;
 b5=a21; 
 b6=a22; 
 b7=a23; 
 b8=a24;
 
 b9=a25; //REPORTE
 b10=a26; 
 b11=a27; 
 b12=a28;
 b13=a29; 
 b14=a30; 
 b15=a31; 
 b16=a32; 
 
 b17=a33; //M
 b18=a34; 
 b19=a35;
 b20=a36; 
 b21=a37; 
 b22=a38; 
 b23=a39;
 b24=a40; 
 
 b25=a41; // SIGNO B
 b26=a42; 
 b27=a43;
 b28=a44; 
 b29=a45; 
 b30=a46; 
 b31=a47;
 b32=a48; 

 

 //solo dividir entre 10,100,1000,10000,100000 y sumar al valor de m y b
 s=(b1*10000000)+(b2*1000000)+(b3*100000)+(b4*10000)+(b5*1000)+(b6*100)+(b7*10)+(b8);
 s=s/10;
 s=ceil(s);//redondea a entero

      
//b8 es el dato asc letra
 r=(b9*10000000)+(b10*1000000)+(b11*100000)+(b12*10000)+(b13*1000)+(b14*100)+(b15*10)+(b16);
 r=r/10;
 r=ceil(r);//redondea a entero


 m=(b17/10)+(b18/100)+(b19/1000)+(b20/10000)+(b21/100000)+(b22/1000000)+(b23/10000000)+(b24/100000000);
 b=(b26/10)+(b27/100)+(b28/1000)+(b29/10000)+(b30/100000)+(b31/1000000)+(b32/10000000);
 //cambio el signo
 if(b25==1){
          b=-1*b;
            } 
 //si entra si esta en ceRo solo que no esta bien debe estar a tierra  
 //manana voy a poner la rutina de comp y de DATOS
 
 ini:
         if(input(VBAJO)){ //INDICA VBATERIA MENOR A 6 VOLTIOS
                          mensaje=1;
                         }
                         else{mensaje=0; }
            if (mensaje==1){
                             lcd_gotoxy(1,1);
                             printf(lcd_putc,"!!BATERIA BAJA!!");//imprimo rpm
                             lcd_gotoxy(1,1);
                             delay_ms(500);
                             printf(lcd_putc,"                ");//imprimo rpm
                             
                             delay_ms(500);
                             lcd_gotoxy(1,1);
                             printf(lcd_putc,"!!BATERIA BAJA!!");//imprimo rpm
                             lcd_gotoxy(1,1);
                             delay_ms(500);
                             printf(lcd_putc,"                ");//imprimo rpm
                             
                             delay_ms(500);
                             lcd_gotoxy(1,1);
                             printf(lcd_putc,"!!BATERIA BAJA!!");//imprimo rpm
                             lcd_gotoxy(1,1);
                             delay_ms(500);
                             printf(lcd_putc,"                ");//imprimo rpm
                             delay_ms(500);
                            }
  
  //MZO 2011 PARA VER S Y R
   lcd_gotoxy(1,1);
   printf(lcd_putc,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16);//imprimo rpm
   lcd_gotoxy(1,2);
   printf(lcd_putc,"S:%6.0f",s);//imprimo rpm
   lcd_gotoxy(9,2);
   printf(lcd_putc,"R:%6.0f",r);//imprimo rpm
   delay_ms(2000);
   lcd_gotoxy(1,1);
   printf(lcd_putc,"                ");//imprimo rpm
   lcd_gotoxy(1,2);
   printf(lcd_putc,"                ");//imprimo rpm

if (med==0){
   lcd_gotoxy(1,1);
   printf(lcd_putc,"m:%1.7f",m);//imprimo rpm
   lcd_gotoxy(1,2);
   printf(lcd_putc,"b:%1.7f",b);//imprimo rpm
           }
llave=0;// si no se ha presionado el boton inicio

usb_task();

 if( usb_attached()){  
                      rutina1();
                      usb_init_cs();
                    }

 
                  while((llave==0)||(med==1)) { //while inicial para entrar al programa
                  //SI INPUT INCIO DEBE HACERSE 3 VECES Y ASIGNAR A UNA VAR VALOR=1,2,3
                  //DESPUES EN UNA RUTINA SWITCH SE DEBE IR A RUTINA 1,2 Y 3BREAK
                  
                     if( ((input(INICIO))&&(via==0))||(med==1) ){//inicio1  medidor
                             lcd_gotoxy(1,1);
                             printf(lcd_putc,"                ");//imprimo rpm
                             lcd_gotoxy(1,1);
                             printf(lcd_putc,"1.MEDIR RPM/VEL ");//imprimo rpm                             
                             via=1;    
                             nTimer1Overflow=0;
                             nTimer1Overflow2=0;
                             delay_ms(2000);
                             
                                                     }//inicio1
                                                     
                                                     
                                                                        
                                                     

                                         
                              if(via==1){//rutina2
                                llave=1;
                                rutina2();//medir y luego rutina 3 ver registro si via==2
                                        } //rutina2
                           
              
                           
                                  if(via==2){//rutina3
                                            rutina3();//solo si se presiona rapido el boton 2 veces
                                            } //rutina3        
                           
                                     
                                  } //while

        


   }//main

