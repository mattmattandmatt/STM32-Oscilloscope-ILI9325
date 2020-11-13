/*
See rights and use declaration in License.h
This library has been modified for the Maple Mini
*/

#ifndef _ADAFRUIT_ILI9325H_
#define _ADAFRUIT_ILI9325H_

#include "Arduino.h"
#include "Print.h"
#include <Adafruit_GFX.h>
#include <avr/pgmspace.h>

#define ILI9325_TFTWIDTH  240 // default to portrait mode
#define ILI9325_TFTHEIGHT 320
/*
#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0xD3
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0D
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
*/
/*
#define ILI9341_PWCTR6  0xFC
*/

// These are copied from Arduino
#define ILI932X_START_OSC          0x00
#define ILI932X_DRIV_OUT_CTRL      0x01
#define ILI932X_DRIV_WAV_CTRL      0x02
#define ILI932X_ENTRY_MOD          0x03
#define ILI932X_RESIZE_CTRL        0x04
#define ILI932X_16BITS_FORMAT      0x05 // Added - might need to remove
#define ILI932X_DISP_CTRL1         0x07
#define ILI932X_DISP_CTRL2         0x08
#define ILI932X_DISP_CTRL3         0x09
#define ILI932X_DISP_CTRL4         0x0A
#define ILI932X_RGB_DISP_IF_CTRL1  0x0C
#define ILI932X_FRM_MARKER_POS     0x0D
#define ILI932X_RGB_DISP_IF_CTRL2  0x0F
#define ILI932X_POW_CTRL1          0x10
#define ILI932X_POW_CTRL2          0x11
#define ILI932X_POW_CTRL3          0x12
#define ILI932X_POW_CTRL4          0x13
#define ILI932X_GRAM_HOR_AD        0x20
#define ILI932X_GRAM_VER_AD        0x21
#define ILI932X_RW_GRAM            0x22
#define ILI932X_POW_CTRL7          0x29
#define ILI932X_FRM_RATE_COL_CTRL  0x2B
#define ILI932X_GAMMA_CTRL1        0x30
#define ILI932X_GAMMA_CTRL2        0x31
#define ILI932X_GAMMA_CTRL3        0x32
#define ILI932X_GAMMA_CTRL4        0x35
#define ILI932X_GAMMA_CTRL5        0x36
#define ILI932X_GAMMA_CTRL6        0x37
#define ILI932X_GAMMA_CTRL7        0x38
#define ILI932X_GAMMA_CTRL8        0x39
#define ILI932X_GAMMA_CTRL9        0x3C
#define ILI932X_GAMMA_CTRL10       0x3D
#define ILI932X_HOR_START_AD       0x50
#define ILI932X_HOR_END_AD         0x51
#define ILI932X_VER_START_AD       0x52
#define ILI932X_VER_END_AD         0x53
#define ILI932X_GATE_SCAN_CTRL1    0x60 // Driver Output Control 2
#define ILI932X_GATE_SCAN_CTRL2    0x61 // Base Image Display Control
#define ILI932X_GATE_SCAN_CTRL9    0x66 // SPI Read/Write Control
#define ILI932X_GATE_SCAN_CTRL3    0x6A // Vertical Scroll Control
#define ILI932X_PART_IMG1_DISP_POS 0x80
#define ILI932X_PART_IMG1_START_AD 0x81
#define ILI932X_PART_IMG1_END_AD   0x82
#define ILI932X_PART_IMG2_DISP_POS 0x83
#define ILI932X_PART_IMG2_START_AD 0x84
#define ILI932X_PART_IMG2_END_AD   0x85
#define ILI932X_PANEL_IF_CTRL1     0x90
#define ILI932X_PANEL_IF_CTRL2     0x92
#define ILI932X_PANEL_IF_CTRL3     0x93
#define ILI932X_PANEL_IF_CTRL4     0x95
#define ILI932X_PANEL_IF_CTRL5     0x97
#define ILI932X_PANEL_IF_CTRL6     0x98 // Unused ?
#define ILI932X_DEEP_STANDBY_CTRL  0xE6
#define TFTLCD_DELAY               0xFF


static const uint16_t ILI932x_regValues[] = {
  ILI932X_START_OSC        , 0x0001, // Start oscillator ? / NoOp / Read-ID
  TFTLCD_DELAY             , 50,     // 50 millisecond delay
  ILI932X_DRIV_OUT_CTRL    , 0x0100, // can Flip Landscape - 0000 0x0x 0000 0000
  ILI932X_DRIV_WAV_CTRL    , 0x0700, // 0x0700
  ILI932X_ENTRY_MOD        , 0x1008, // xx08=Land1 xx38=Land2 xx20=Port1 xx10=Port2-conn-side    xx18=Land-Flip xx28=Land-Flip-Mirr - will be over-ridden
  ILI932X_RESIZE_CTRL      , 0x0000, // 0x0
  ILI932X_16BITS_FORMAT    , 0x0002,
  0x06                     , 0x0,
  ILI932X_DISP_CTRL1       , 0x0,
  ILI932X_DISP_CTRL2       , 0x0202, // front porch, back porch
  ILI932X_DISP_CTRL4       , 0x0,
  0x0B                     , 0x0,
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,
  ILI932X_FRM_MARKER_POS   , 0x0,
  0x0E                     , 0x0,
  ILI932X_RGB_DISP_IF_CTRL2, 0x0, // 0x0
  ILI932X_POW_CTRL1        , 0x0,
  ILI932X_POW_CTRL2        , 0x0007,
  ILI932X_POW_CTRL3        , 0x0,
  ILI932X_POW_CTRL4        , 0x0,
  TFTLCD_DELAY             , 200,
  0x14                     , 0x0,
  0x15                     , 0x0,
  0x16                     , 0x0,
  0x17                     , 0x0,
  0x18                     , 0x0,
  0x19                     , 0x0,
  0x1A                     , 0x0,
  0x1B                     , 0x0,
  0x1C                     , 0x0,
  0x1D                     , 0x0,
  0x1E                     , 0x0,
  0x1F                     , 0x0,
  ILI932X_POW_CTRL1        , 0x1690,
  ILI932X_POW_CTRL2        , 0x0227, // 0x0227
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL3        , 0x001A, // 0x1A
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL4        , 0x1800,
  ILI932X_POW_CTRL7        , 0x002A,
  TFTLCD_DELAY             , 50,
  ILI932X_GAMMA_CTRL1      , 0x0000,
  ILI932X_GAMMA_CTRL2      , 0x0000,
  ILI932X_GAMMA_CTRL3      , 0x0000,
  ILI932X_GAMMA_CTRL4      , 0x0206, // 0x0206
  ILI932X_GAMMA_CTRL5      , 0x0808,
  ILI932X_GAMMA_CTRL6      , 0x0007,
  ILI932X_GAMMA_CTRL7      , 0x0201,
  ILI932X_GAMMA_CTRL8      , 0x0000,
  ILI932X_GAMMA_CTRL9      , 0x0000,
  ILI932X_GAMMA_CTRL10     , 0x0000,
  ILI932X_GRAM_HOR_AD      , 0x0000,
  ILI932X_GRAM_VER_AD      , 0x0000,
  ILI932X_HOR_START_AD     , 0x0000,
  ILI932X_HOR_END_AD       , ILI9325_TFTHEIGHT - 1,
  ILI932X_VER_START_AD     , 0x0000,
  ILI932X_VER_END_AD       , ILI9325_TFTWIDTH - 1,
  ILI932X_GATE_SCAN_CTRL1  , 0xA700, // Driver Output Control (R60h) // 0xA700
  ILI932X_GATE_SCAN_CTRL2  , 0x0003, // Driver Output Control (R61h) // 0x0003
  ILI932X_GATE_SCAN_CTRL3  , 0x0000, // Driver Output Control (R62h)
  ILI932X_PANEL_IF_CTRL1   , 0x0010, // Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2   , 0x0000,
  ILI932X_PANEL_IF_CTRL3   , 0x0003,
  ILI932X_PANEL_IF_CTRL4   , 0x1100,
  ILI932X_PANEL_IF_CTRL5   , 0x0000,
  ILI932X_PANEL_IF_CTRL6   , 0x0000,
  ILI932X_DISP_CTRL1       , 0x0133, // Main screen turn on
};


// Color definitions
#define ILI9325_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9325_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9325_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9325_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9325_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9325_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9325_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9325_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9325_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9325_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9325_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9325_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9325_RED         0xF800      /* 255,   0,   0 */
#define ILI9325_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9325_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9325_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9325_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9325_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9325_PINK        0xF81F

/*
Define pins and Output Data Registers
*/

//Port data |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
//Pin stm32 |PA7|PA6|PA5|PA4|PA3|PA2|PC1|PA0|
//Control pins |RD |WR |RS |CS |RST|
//Pin stm32    |PB4|PB5|PB6|PB7|PB8|
#define TFT_CNTRL      GPIOB
#define TFT_DATA       GPIOA
#define TFT_RD         PB4
#define TFT_WR         PB5
#define TFT_RS         PB6
#define TFT_CS         PB7
#define TFT_RST        PB8
#define TFT_RD_MASK    digitalPinToBitMask(TFT_RD)
#define TFT_WR_MASK    digitalPinToBitMask(TFT_WR)
#define TFT_RS_MASK    digitalPinToBitMask(TFT_RS)
#define TFT_CS_MASK    digitalPinToBitMask(TFT_CS)

#define RD_ACTIVE    TFT_CNTRL->regs->BRR  = TFT_RD_MASK
#define RD_IDLE      TFT_CNTRL->regs->BSRR = TFT_RD_MASK
#define WR_ACTIVE    TFT_CNTRL->regs->BRR  = TFT_WR_MASK
#define WR_IDLE      TFT_CNTRL->regs->BSRR = TFT_WR_MASK
#define CD_COMMAND   TFT_CNTRL->regs->BRR  = TFT_RS_MASK
#define CD_DATA      TFT_CNTRL->regs->BSRR = TFT_RS_MASK
#define CS_ACTIVE    TFT_CNTRL->regs->BRR  = TFT_CS_MASK
#define CS_IDLE      TFT_CNTRL->regs->BSRR  = TFT_CS_MASK

#ifndef RD_STROBE
 #define RD_STROBE  {RD_ACTIVE; RD_IDLE;}
#endif
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }
#define swap(a, b) { int16_t t = a; a = b; b = t; }

//Set pins to the 8 bit number
#define write8special(c) { TFT_DATA->regs->BSRR = ((~c)<<16) | (c); WR_STROBE; }


class Adafruit_ILI9325_8bit_STM : public Adafruit_GFX {

 public:

  Adafruit_ILI9325_8bit_STM(void);
  
  void     begin(void),
           setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
		   #if defined (__STM32F1__)
		   drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color),
		   #endif
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
             uint16_t color),
           setRotation(uint8_t r);
//           invertDisplay(boolean i);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  /* These are not for current use, 8-bit protocol only! */
  //uint8_t  readdata(void),
   uint8_t readcommand8(uint8_t reg); 
   uint32_t readID(void);


 private:
  uint8_t  tabcolor;
  uint8_t  read8(void);
  void     setReadDataBus(void),
    setWriteDataBus(void),
    write8(uint8_t),
    writecommand(uint8_t c),
    writedata(uint8_t d),
    writecommand16(uint16_t c),
    writedata16(uint16_t d),
    commandList(uint8_t *addr);




#if defined (__STM32F1__)
	 uint16_t lineBuffer[ILI9325_TFTHEIGHT]; // DMA buffer. 16bit color data per pixel
#endif
};

#endif //endif of the header file
