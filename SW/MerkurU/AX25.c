#nolist
//#define  PTT PIN_A2                 // PTT control
//#define  TXo PIN_C0                 // To the transmitter modulator
#define  PERIODAH delay_us(222)     // Halfperiod H 222;78/1200     500;430/500
#define  TAILH delay_us(78)
#define  PERIODAL delay_us(412)     // Halfperiod L 412;345/1200    1000;880/500
#define  TAILL delay_us(345)
#byte    STATUS = 3                 // CPUs status register

byte SendData[16] = {'A'<<1, 'L'<<1, 'L'<<1, ' '<<1, ' '<<1, ' '<<1,  0x60,
                     'C'<<1, 'Z'<<1, '0'<<1, 'R'<<1, 'R'<<1, 'R'<<1, 0x61,
                     0x03, 0xF0};

boolean bit;
int fcslo, fcshi;    // variabloes for calculating FCS (CRC)
int stuff;           // stuff counter for extra 0
int flag_flag;       // if it is sending flag (7E)
int fcs_flag;        // if it is sending Frame Check Sequence
int i;               // for for

void flipout()       //flips the state of output pin a_1
{
	stuff = 0;        //since this is a 0, reset the stuff counter
	if (bit)
   {
     bit=FALSE;      //if the state of the pin was low, make it high.
   }
   else
   {
     bit=TRUE;	  	   //if the state of the pin was high make it low
   }
}

void fcsbit(byte tbyte)
{
#asm
   BCF    STATUS,0
   RRF    fcshi,F             // rotates the entire 16 bits
   RRF    fcslo,F			      // to the right
#endasm
   if (((STATUS & 0x01)^(tbyte)) ==0x01)
   {
         fcshi = fcshi^0x84;
         fcslo = fcslo^0x08;
   }
}

void SendBit ()
{
	if (bit)
   {
      output_high(TXo);
      PERIODAH;
      output_low(TXo);
      PERIODAH;
      output_high(TXo);
      PERIODAH;
      output_low(TXo);
      TAILH;
    }
    else
    {
      output_high(TXo);
      PERIODAL;
      output_low(TXo);
      TAILL;
    };
}

void SendByte (byte inbyte)
{
   int k, bt;

   for (k=0;k<8;k++)    //do the following for each of the 8 bits in the byte
   {
     bt = inbyte & 0x01;            //strip off the rightmost bit of the byte to be sent (inbyte)
     if ((fcs_flag == FALSE) & (flag_flag == FALSE)) fcsbit(bt);    //do FCS calc, but only if this
						                                                //is not a flag or fcs byte
     if (bt == 0)
     {
       flipout();
     }  			               // if this bit is a zero, flip the output state
     else
     {                          			      //otherwise if it is a 1, do the following:
       if (flag_flag == FALSE) stuff++;      //increment the count of consequtive 1's
       if ((flag_flag == FALSE) & (stuff == 5))
       {       //stuff an extra 0, if 5 1's in a row
         SendBit();
         flipout();               //flip the output state to stuff a 0
       }//end of if
     }//end of else
     // delay_us(850);				 //introduces a delay that creates 1200 baud
     SendBit();
     inbyte = inbyte>>1;          //go to the next bit in the byte
   }//end of for
}//end of SendByte

void SendPacket(char *data)
{
    bit=FALSE;

   fcslo=fcshi=0xFF;       //The 2 FCS Bytes are initialized to FF
   stuff = 0;              //The variable stuff counts the number of 1's in a row. When it gets to 5
		                     // it is time to stuff a 0.

//   output_low(PTT);        // Blinking LED
//   delay_ms(1000);
//   output_high(PTT);

   flag_flag = TRUE;       //The variable flag is true if you are transmitted flags (7E's) false otherwise.
   fcs_flag = FALSE;       //The variable fcsflag is true if you are transmitting FCS bytes, false otherwise.

   for(i=0; i<10; i++) SendByte(0x7E); //Sends flag bytes.  Adjust length for txdelay
                                       //each flag takes approx 6.7 ms
   flag_flag = FALSE;      //done sending flags

   for(i=0; i<16; i++) SendByte(SendData[i]);      //send the packet bytes

   for(i=0; 0 != *data; i++)
   {
      SendByte(*data);     //send the packet bytes
      data++;
   };

   fcs_flag = TRUE;       	//about to send the FCS bytes
   fcslo =fcslo^0xff;      //must XOR them with FF before sending
   fcshi = fcshi^0xff;
   SendByte(fcslo);        //send the low byte of fcs
   SendByte(fcshi);        //send the high byte of fcs
   fcs_flag = FALSE;		   //done sending FCS
   flag_flag = TRUE;  		//about to send flags
   SendByte(0x7e);         // Send a flag to end packet
}

#list
