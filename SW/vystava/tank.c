#include "tank.h"

unsigned int8 sensors;        // pomocna promenna pro cteni cidel na caru
unsigned int8 line;           // na ktere strane byla detekovana cara
unsigned int8 speed;          // rychlost zataceni
unsigned int8 last;           // kde byla cara, kdyz byly minule cidla mimo
unsigned int8 rovinka;        // pocitadlo pro zjisteni rovneho useku
int cirkus;

// Konstanty pro dynamiku pohybu
#define T_DIRA       120      // po jakem case zataceni se detekuje dira
#define FW_POMALU    170      // trochu mimo caru vnitrni pas
#define FW_ZATACKA   200      // rychlost vnejsiho kola pri zataceni
#define FW_STREDNE   190      // trochu mimo caru vnejsi pas
#define COUVANI      750      // couvnuti zpet na caru, po detekci diry
#define MAX_ROVINKA  (255-FW_STREDNE)
#define TRESHOLD     15        // rozhodovaci uroven komparatoru, 0xF = 0.75*Vdd
#define BUMPER_TRESHOLD 128

//motory            //Napred vypnout potom zapnout!
#define FR         output_low(PIN_B5); output_high(PIN_B4)  // Vpred
#define FL         output_low(PIN_B7); output_high(PIN_B6)
#define BR         output_low(PIN_B4); output_high(PIN_B5)  // Vzad
#define BL         output_low(PIN_B6); output_high(PIN_B7)
#define STOPR      output_low(PIN_B4);output_low(PIN_B5)    // Zastav
#define STOPL      output_low(PIN_B6);output_low(PIN_B7)

#define L 0b10  // left
#define R 0b01  // right
#define S 0b11  // straight

//cidla
#define RSENSOR    !C2OUT      // Senzory na caru
#define LSENSOR    !C1OUT
#define BUMPER     PIN_A4     // Senzor na cihlu

#define DIAG_SERVO      PIN_B3   // Propojka pro diagnosticky mod
#define DIAG_SENSORS    PIN_B2   // Propojka pro diagnosticky mod

#DEFINE SOUND_HI   PIN_A6     // komplementarni vystupy pro piezo pipak
#DEFINE SOUND_LO   PIN_A7

char AXstring[40];   // Buffer pro prenos telemetrie

// makro pro PWM
#define GO(motor, direction, power) if(get_timer0()<=power) \
{direction##motor;} else {stop##motor;}

#int_TIMER2
void TIMER2_isr()
{
   if (speed<255) speed++;
   if (rovinka<MAX_ROVINKA) rovinka++;
}
// Primitivni Pipani
void beep(unsigned int16 period, unsigned int16 length)
{
   unsigned int16 nn;

   for(nn=length; nn>0; nn--)
   {
     output_high(SOUND_HI);output_low(SOUND_LO);
     delay_us(period);
     output_high(SOUND_LO);output_low(SOUND_HI);
     delay_us(period);
   }
}
/******************************************************************************/
void diagnostika()
{
   unsigned int16 n;

   while (input(DIAG_SERVO))   // Propojka, ktera spousti diagnostiku
   {
      for (n=500; n<800; n+=100)
      {
         beep(n,n); //beep UP
      };
      Delay_ms(1000);
      //zastav vse
      STOPL; STOPR;
      //pravy pas
      FR; Delay_ms(1000); STOPR; Delay_ms(1000);
      BR; Delay_ms(1000); STOPR; Delay_ms(1000);
      Beep(880,100); Delay_ms(1000);
      //levy pas
      FL; Delay_ms(1000); STOPL; Delay_ms(1000);
      BL; Delay_ms(1000); STOPL; Delay_ms(1000);
      Beep(880,100); Delay_ms(1000);
      //oba pasy
      FL; FR; Delay_ms(1000); STOPL; STOPR; Delay_ms(1000);
      BL; BR; Delay_ms(1000); STOPL; STOPR; Delay_ms(1000);
   };
   while (input(DIAG_SENSORS))         // spusteni diagnostiky cidel
   {
      if (RSENSOR) beep(1000,1000);
      if (LSENSOR) beep(2000,2000);
      if ((read_adc(ADC_READ_ONLY)<BUMPER_TRESHOLD)) beep(3000,3000);
   };
}
///////////////////////////////////////////////////////////////////////////////
void OtocSe()                     // otoci se zpet, kdyz je prekazka
{
unsigned int16 n;

	BR;BL;
	beep(800,400);
	beep(2000,1000);
	beep(900,400);

   BR; FL; Delay_ms(50);           // otoc se 30� do prava
   STOPL; STOPR;
   beep(1000,1000);

   BR; FL;
   for(n=40000;n>0;n--)               // toc se, dokud nenarazis na caru
   {
      line = RSENSOR;         // cteni senzoru na caru
      line |= LSENSOR << 1;
      if (line!=0) break;
   }
   STOPR; STOPL;

   line=L; // caru jsme prejeli, tak je vlevo
   cirkus=0;
}


void main()
{
   unsigned int16 n; // pro FOR
   unsigned int16 i;

   STOPL; STOPR;     // prepne vystupy na ovladani motoru na output a zastavi

   setup_oscillator(OSC_4MHZ|OSC_INTRC);     // 4 MHz interni RC oscilator

   port_b_pullups(TRUE);      // pullups pro piano na diagnostiku
   setup_spi(FALSE);
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);  // Casovac pro PWM

   setup_timer_2(T2_DIV_BY_4,255,10);    // Casovac pro regulaci
                                         // preruseni kazdych 10ms
   setup_adc_ports(sAN2|VSS_VDD);      // nastaveni A/D prevodniku pro naraznik
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(2);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);   // Casovac pro naraznik
   setup_ccp1(CCP_COMPARE_RESET_TIMER);
   CCP_1=(2^10)-1;                        // prevod kazdou 1ms

   setup_comparator(A0_VR_A1_VR);   // inicializace komparatoru pro cidla cary
   setup_vref(VREF_HIGH|TRESHOLD);        // 32 kroku od 0.25 do 0.75 Vdd

   Beep(1000,200);     //double beep
   Delay_ms(50);
   Beep(1000,200);
   Delay_ms(1000);      // 1s

   // povoleni rizeni rychlosti zataceni pres preruseni
   enable_interrupts(INT_TIMER2);
   enable_interrupts(GLOBAL);

/*---------------------------------------------------------------------------*/
   sensors=S;
   line=S;
   last=S;
   cirkus=0;
//   movement=S;
   speed=FW_POMALU;

   diagnostika();
   Delay_ms(500);
   Beep(1000,200);
   Delay_ms(500);

   while(true)       // hlavni smycka (jizda podle cary)
   {
      sensors = RSENSOR;         // cteni senzoru na caru
      sensors |= LSENSOR << 1;

      if (read_adc(ADC_READ_ONLY)<BUMPER_TRESHOLD) OtocSe();

      switch (sensors)  // zatacej podle toho, kde vidis caru
      {
         case S:                          // rovne
            GO(L, F, FW_STREDNE+rovinka); GO(R, F, FW_STREDNE+rovinka);
            continue;
         case L:                          // trochu vlevo
            GO(L, F, FW_POMALU+rovinka); GO(R, F, FW_STREDNE+rovinka);
            line=L;
            continue;
         case R:                          // trochu vpravo
            GO(R, F, FW_POMALU+rovinka); GO(L, F, FW_STREDNE+rovinka);
            line=R;
            continue;
         default:       // kdyz jsou obe cidla mimo caru, tak pokracuj dal
      };
      rovinka=0;

      if (last!=line)     // pokud si prejel caru z jedne strany na druhou stranu, tak zabrzdi
      {
         last=line;
         speed=FW_ZATACKA;
         cirkus++;
         if (cirkus>10)
         {
            STOPL; STOPR;
            cirkus=0;
            disable_interrupts(GLOBAL);
            beep(1000,400);
            for(n=3000; n>3950; n--) beep(n,10);
      	   beep(2000,200);
	         beep(900,400);
            for(n=2950; n<3000; n++) beep(n,10);
            beep(4000,400);
      	   beep(1000,100);
	         beep(3000,400);
            Delay_ms(1000);
            enable_interrupts(GLOBAL);
         }
      };

      if (L==line)  // kdyz jsou obe cidla mimo caru, zatoc na caru
      {
         STOPL;
         GO(R, F, speed);
      }
      else
      {
         STOPR;
         GO(L, F, speed);
      }
   } // while(true)
}
