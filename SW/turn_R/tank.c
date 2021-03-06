#include "tank.h"

#define DEBUG

#define  TXo PIN_A3           // To the transmitter modulator
#include "AX25.c"             // podprogram pro prenos telemetrie

unsigned int8 sensors;        // pomocna promenna pro cteni cidel na caru
unsigned int8 line;           // na ktere strane byla detekovana cara
unsigned int8 speed;          // rychlost zataceni
unsigned int8 rovinka;        // pocitadlo pro zjisteni rovneho useku
unsigned int8 last;           // kde byla cara, kdyz byly minule cidla mimo
unsigned int8 movement;       // obsahuje aktualni smer zataceni
unsigned int8 dira;           // pocita dobu po kterou je ztracena cara

// Konstanty pro dynamiku pohybu
#define T_DIRA       120      // po jakem case zataceni se detekuje dira
#define INC_SPEED    1        // prirustek rychlosti v jednom kroku
#define FW_POMALU    230      // trochu mimo caru vnitrni pas
#define FW_ZATACKA   240      // rychlost vnejsiho kola pri zataceni
#define FW_STREDNE   240      // trochu mimo caru vnejsi pas
#define COUVANI      750      // couvnuti zpet na caru, po detekci diry
#define PRES_DIRU    300
#define MAX_ROVINKA  (255-FW_STREDNE)
#define TRESHOLD     6        // rozhodovaci uroven komparatoru, 0xF = 0.75*Vdd
#define BUMPER_TRESHOLD 128
#define CIK_CAK      30000
#define T_CIHLA      50      // perioda detekce cihly

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
#define RSENSOR    C2OUT      // Senzory na caru
#define LSENSOR    C1OUT
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
   if (speed<255) speed+=INC_SPEED;
   if (rovinka<MAX_ROVINKA) rovinka++;
   if (dira<=T_DIRA) dira++;
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
      if (RSENSOR) beep(900,500);
      if (LSENSOR) beep(800,500);
      if ((read_adc(ADC_READ_ONLY)<BUMPER_TRESHOLD)) beep(1000,500);
   };
}
///////////////////////////////////////////////////////////////////////////////
void cikcak()
{
unsigned int16 n;
sem1:
   n=CIK_CAK;
	while (0==RSENSOR||LSENSOR)       // zkontroluj caru
	{
		if (n==CIK_CAK)							// zmen smer zataceni
		{
			n=0;
			switch(movement)
			{
			case L:
						FL;BR;
						movement=R;
					break;
			case R:
						FR;BL;
						movement=L;
					break;
			case S:
						FL;BR;
						movement=R;
						n=CIK_CAK/2;
					break;
			}
		}
		n++;
	}
	STOPL;STOPR;
	line = RSENSOR;         // cteni senzoru na caru
	line |= LSENSOR << 1;
	if (line==0) goto sem1;
   												// nasli jsme caru
	line=S;
}
///////////////////////////////////////////////////////////////////////////////
void objizdka()                     // objede cihlu
{
unsigned int16 n;

	BL;BR;Delay_ms(150);
	STOPR;STOPL;
	beep(900,1000);
  // movement=S;
  // cikcak();

   BR; FL; Delay_ms(270);           // otoc se 70� do prava

   FR; FL; Delay_ms(500);           // popojed rovne

   BL; Delay_ms(30);               // otoc se 90� do leva
   STOPL; FR; Delay_ms(500);

   FR; FL;  Delay_ms(100);           // popojed rovne na slepo
   for(n=40000;n>0;n--)               // popojed rovne ale kontroluj caru
   {
      line = RSENSOR;         // cteni senzoru na caru
      line |= LSENSOR << 1;
      if (line!=0) {Delay_ms(150); break;}
//     Delay_ms(1);
   }

   BR; FL;                             // otoc se 60� do prava
   for(n=40000;n>0;n--)
   {
      line = RSENSOR;         // cteni senzoru na caru
      line |= LSENSOR << 1;
      if (line!=0) break;
//      Delay_ms(1);
   }
   STOPR; STOPL;

   movement=L; //R;
   cikcak();
   dira=0;
}
///////////////////////////////////////////////////////////////////////////////
void prejeddiru()                    // vyresi diru
{
unsigned int16 n;
unsigned int8 speed_dira;

   STOPL;STOPR;
   speed_dira=speed;
   beep(1000,500);
   switch (movement)                            //vrat se zpet na caru
   {
   case L:
         for (n=COUVANI;n>0;n--) {GO(R,B,speed_dira); Delay_ms(1);}
         STOPL;STOPR;
      break;
   case R:
         for (n=COUVANI;n>0;n--) {GO(L,B,speed_dira); Delay_ms(1);}
         STOPL;STOPR;
      break;
   case S:
         goto sem;
      break;
   }
   beep(1000,500);

   /*line=0;
   FR; BL; Delay_ms(400);                 // otoc se na caru
   beep(1000,500);
   while(line==0)
   {
      line = RSENSOR;         // cteni senzoru na caru
      line |= LSENSOR << 1;
   }
   FL;BR; Delay_ms(60);       // zabrzdi
   STOPL; STOPR;

   FL; BR; Delay_ms(700);     // otacka 180 deg
   STOPL; STOPR;*/

   FR;FL;                     //popojed rovne
   for(n=PRES_DIRU;n>0;n--)
   {
      line = RSENSOR;         // cteni senzoru na caru
      line |= LSENSOR << 1;
      if (line!=0) break;
      Delay_ms(1);
   }
sem:
   STOPL; STOPR;
   movement=S;
   cikcak();                                    // najdi caru
   dira=0;
}
///////////////////////////////////////////////////////////////////////////////
void main()
{
   unsigned int16 n; // pro FOR

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
   movement=S;
   speed=FW_POMALU;

   diagnostika();
   //cikcak();     // toc se, abys nasel caru
   Delay_ms(500);
   Beep(1000,200);
   Delay_ms(500);

   while(true)       // hlavni smycka (jizda podle cary)
   {
      sensors = RSENSOR;         // cteni senzoru na caru
      sensors |= LSENSOR << 1;

      if ((read_adc(ADC_READ_ONLY)<BUMPER_TRESHOLD) && (dira<=T_CIHLA)) objizdka();

      switch (sensors)  // zatacej podle toho, kde vidis caru
      {
         case S:                          // rovne
            FL; FR;  // pokud se jede dlouho rovne, tak pridej
            dira=0;
            movement=S;
            continue;
         case L:                          // trochu vlevo
            GO(L, F, FW_POMALU+rovinka); GO(R, F, FW_STREDNE+rovinka);
            line=L;
            dira=0;
            movement=L;
            continue;
         case R:                          // trochu vpravo
            GO(R, F, FW_POMALU+rovinka); GO(L, F, FW_STREDNE+rovinka);
            line=R;
            dira=0;
            movement=R;
            continue;
         default:       // kdyz jsou obe cidla mimo caru, tak pokracuj dal
      }
   rovinka=0;
      if (dira>=T_DIRA) prejeddiru();
      if (last!=line)     // pokud si prejel caru z jedne strany na druhou stranu, tak zabrzdi
      {
         last=line;
         speed=FW_ZATACKA;
      }
      if (L==line)  // kdyz jsou obe cidla mimo caru, zatoc na caru
      {
         STOPL;
         GO(R, F, speed);
         movement=L;
      }
      else
      {
         STOPR;
         GO(L, F, speed);
         movement=R;
      }
   } // while(true)
}

