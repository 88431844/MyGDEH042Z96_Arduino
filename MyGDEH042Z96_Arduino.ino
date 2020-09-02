#include"Ap_29demo.h"
//IO settings
int BUSY_Pin = 14; 
int RES_Pin = 12; 
int DC_Pin = 13; 
int CS_Pin = 15; 
int SCK_Pin = 3; 
int SDI_Pin = 1; 
#define EPD_W21_MOSI_0  digitalWrite(SDI_Pin,LOW)
#define EPD_W21_MOSI_1  digitalWrite(SDI_Pin,HIGH) 

#define EPD_W21_CLK_0 digitalWrite(SCK_Pin,LOW)
#define EPD_W21_CLK_1 digitalWrite(SCK_Pin,HIGH)

#define EPD_W21_CS_0 digitalWrite(CS_Pin,LOW)
#define EPD_W21_CS_1 digitalWrite(CS_Pin,HIGH)

#define EPD_W21_DC_0  digitalWrite(DC_Pin,LOW)
#define EPD_W21_DC_1  digitalWrite(DC_Pin,HIGH)
#define EPD_W21_RST_0 digitalWrite(RES_Pin,LOW)
#define EPD_W21_RST_1 digitalWrite(RES_Pin,HIGH)
#define isEPD_W21_BUSY digitalRead(BUSY_Pin)
//400*300///////////////////////////////////////

#define MONOMSB_MODE 1
#define MONOLSB_MODE 2 
#define RED_MODE     3

#define MAX_LINE_BYTES   50//=400/8
#define MAX_COLUMN_BYTES  300

#define ALLSCREEN_GRAGHBYTES  15000

////////FUNCTION//////
void driver_delay_us(unsigned int xus);
void driver_delay_xms(unsigned long xms);
void DELAY_S(unsigned int delaytime);     
void SPI_Delay(unsigned char xrate);
void SPI_Write(unsigned char value);
void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char command);
//EPD
void Epaper_READBUSY(void);
void Epaper_Spi_WriteByte(unsigned char TxData);
void Epaper_Write_Command(unsigned char cmd);
void Epaper_Write_Data(unsigned char data);

void EPD_HW_Init(void); //Electronic paper initialization
void EPD_Update(void);
void EPD_DeepSleep(void);
//Display 
void EPD_WhiteScreen_ALL(const unsigned char *BW_datas,const unsigned char *R_datas);
void EPD_WhiteScreen_ALL_Clean(void);
void setup() {
   pinMode(BUSY_Pin, INPUT); 
   pinMode(RES_Pin, OUTPUT);  
   pinMode(DC_Pin, OUTPUT);    
   pinMode(CS_Pin, OUTPUT);    
   pinMode(SCK_Pin, OUTPUT);    
   pinMode(SDI_Pin, OUTPUT);    
}

//Tips//
/*When the electronic paper is refreshed in full screen, the picture flicker is a normal phenomenon, and the main function is to clear the display afterimage in the previous picture.
  When the local refresh is performed, the screen does not flash.*/
/*When you need to transplant the driver, you only need to change the corresponding IO. The BUSY pin is the input mode and the others are the output mode. */
  
void loop() {
  while(1)
  { 
    //Full screen refresh
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_BW,gImage_R); //Refresh the picture in full screen   
    EPD_DeepSleep(); //Enter deep sleep,Sleep instruction is necessary, please do not delete!!! 
    delay(3000);   
    
    //Clean
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL_Clean();
    EPD_DeepSleep(); //Enter deep sleep,Sleep instruction is necessary, please do not delete!!!
    while(1);
  }
}











///////////////////EXTERNAL FUNCTION////////////////////////////////////////////////////////////////////////
/////////////////////delay//////////////////////////////////////
void driver_delay_us(unsigned int xus)  //1us
{
  for(;xus>1;xus--);
}
void driver_delay_xms(unsigned long xms) //1ms
{  
    unsigned long i = 0 , j=0;

    for(j=0;j<xms;j++)
  {
        for(i=0; i<256; i++);
    }
}
void DELAY_S(unsigned int delaytime)     
{
  int i,j,k;
  for(i=0;i<delaytime;i++)
  {
    for(j=0;j<4000;j++)           
    {
      for(k=0;k<222;k++);
                
    }
  }
}
//////////////////////SPI///////////////////////////////////
void SPI_Delay(unsigned char xrate)
{
  unsigned char i;
  while(xrate)
  {
    for(i=0;i<2;i++);
    xrate--;
  }
}

void SPI_Write(unsigned char value)                                    
{                                                           
    unsigned char i;  
   SPI_Delay(1);
    for(i=0; i<8; i++)   
    {
        EPD_W21_CLK_0;
       SPI_Delay(1);
       if(value & 0x80)
          EPD_W21_MOSI_1;
        else
          EPD_W21_MOSI_0;   
        value = (value << 1); 
       SPI_Delay(1);
       driver_delay_us(1);
        EPD_W21_CLK_1; 
        SPI_Delay(1);
    }
}

void Epaper_Write_Command(unsigned char command)
{
  SPI_Delay(1);
  EPD_W21_CS_0;                   
  EPD_W21_DC_0;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}
void Epaper_Write_Data(unsigned char command)
{
  SPI_Delay(1);
  EPD_W21_CS_0;                   
  EPD_W21_DC_1;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}

/////////////////EPD settings Functions/////////////////////

void EPD_HW_Init(void)
{
    EPD_W21_RST_0;  // Module reset      
    delay(1); //At least 10ms delay 
    EPD_W21_RST_1; 
    delay(1); //At least 10ms delay  
    
    Epaper_READBUSY();    //waiting for the electronic paper IC to release the idle signal
    Epaper_Write_Command(0x12);     //SWRESET
    Epaper_READBUSY();  //waiting for the electronic paper IC to release the idle signal
  
    Epaper_Write_Command(0x74);
    Epaper_Write_Data(0x54);
    Epaper_Write_Command(0x7E);
    Epaper_Write_Data(0x3B);
    Epaper_Write_Command(0x2B);  // Reduce glitch under ACVCOM  
    Epaper_Write_Data(0x04);           
    Epaper_Write_Data(0x63);

    Epaper_Write_Command(0x0C);  // Soft start setting
    Epaper_Write_Data(0x8B);           
    Epaper_Write_Data(0x9C);
    Epaper_Write_Data(0x96);
    Epaper_Write_Data(0x0F);

    Epaper_Write_Command(0x01);  // Set MUX as 300
    Epaper_Write_Data(0x2B);           
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);     

    Epaper_Write_Command(0x11);  // Data entry mode
    Epaper_Write_Data(0x01);         
    Epaper_Write_Command(0x44); 
    Epaper_Write_Data(0x00); // RAM x address start at 0
    Epaper_Write_Data(0x31); // RAM x address end at 31h(49+1)*8->400
    Epaper_Write_Command(0x45); 
    Epaper_Write_Data(0x2B);   // RAM y address start at 12Bh     
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00); // RAM y address end at 00h     
    Epaper_Write_Data(0x00);
    Epaper_Write_Command(0x3C); // board
    Epaper_Write_Data(0x01); // HIZ

    Epaper_Write_Command(0x18);
    Epaper_Write_Data(0X80);
    Epaper_Write_Command(0x22);
    Epaper_Write_Data(0XB1);  //Load Temperature and waveform setting.
    Epaper_Write_Command(0x20);
    Epaper_READBUSY();    //waiting for the electronic paper IC to release the idle signal
    

    Epaper_Write_Command(0x4E); 
    Epaper_Write_Data(0x00);
    Epaper_Write_Command(0x4F); 
    Epaper_Write_Data(0x2B);
    Epaper_Write_Data(0x01);
}
//////////////////////////////All screen update////////////////////////////////////////////
void EPD_WhiteScreen_ALL(const unsigned char *BW_datas,const unsigned char *R_datas)
{
   unsigned int i;  
  Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
   for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
   {               
     Epaper_Write_Data(pgm_read_byte(&BW_datas[i]));
   }
  Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
   for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
   {               
     Epaper_Write_Data(~pgm_read_byte(&R_datas[i]));
   }
   EPD_Update();   
}

/////////////////////////////////////////////////////////////////////////////////////////
void EPD_Update(void)
{   
  Epaper_Write_Command(0x22); //Display Update Control
  Epaper_Write_Data(0xC7);   
  Epaper_Write_Command(0x20);  //Activate Display Update Sequence
  Epaper_READBUSY();   

}

void EPD_DeepSleep(void)
{  
  Epaper_Write_Command(0x10); //enter deep sleep
  Epaper_Write_Data(0x01); 
  delay(100);
}
void Epaper_READBUSY(void)
{ 
  while(1)
  {   //=1 BUSY
     if(isEPD_W21_BUSY==0) break;
  }  
}

void EPD_WhiteScreen_ALL_Clean(void)
{
   unsigned int i;  
  Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
   for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
   {               
     Epaper_Write_Data(0xff);
   }
  Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
   for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
   {               
     Epaper_Write_Data(0x00);
   }
   EPD_Update();   
}



//////////////////////////////////END//////////////////////////////////////////////////
