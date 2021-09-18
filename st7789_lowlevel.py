import time
import machine
import struct
import uctypes

from micropython import const
import rp2_dma

ST7789_NOP = const(0x00)
ST7789_SWRESET = const(0x01)
ST7789_RDDID = const(0x04)
ST7789_RDDST = const(0x09)

ST7789_SLPIN = const(0x10)
ST7789_SLPOUT = const(0x11)
ST7789_PTLON = const(0x12)
ST7789_NORON = const(0x13)

ST7789_INVOFF = const(0x20)
ST7789_INVON = const(0x21)
ST7789_DISPOFF = const(0x28)
ST7789_DISPON = const(0x29)

ST7789_CASET = const(0x2A)
ST7789_RASET = const(0x2B)
ST7789_RAMWR = const(0x2C)
ST7789_RAMRD = const(0x2E)

ST7789_PTLAR = const(0x30)
ST7789_MADCTL = const(0x36)
ST7789_COLMOD = const(0x3A)

ST7789_WRCACE = const(0x55)

ST7789_FRMCTR1 = const(0xB1)
ST7789_FRMCTR2 = ST7789_PORCTRL = const(0xB2)
ST7789_FRMCTR3 = const(0xB3)
ST7789_INVCTR = const(0xB4)
ST7789_DISSET5 = const(0xB6)

ST7789_GCTRL = const(0xB7)
ST7789_GTADJ = const(0xB8)
ST7789_VCOMS = const(0xBB)

ST7789_LCMCTRL = const(0xC0)
ST7789_IDSET = const(0xC1)
ST7789_VDVVRHEN = const(0xC2)
ST7789_VRHS = const(0xC3)
ST7789_VDVS = const(0xC4)
ST7789_VMCTR1 = const(0xC5)
ST7789_FRCTRL2 = const(0xC6)
ST7789_CABCCTRL = const(0xC7)

ST7789_PWCTRL1 = const(0xD0)
ST7789_RDID1 = const(0xDA)
ST7789_RDID2 = const(0xDB)
ST7789_RDID3 = const(0xDC)
ST7789_RDID4 = const(0xDD)

ST7789_GMCTRP1 = ST7789_PVGAMCTRL = const(0xE0)
ST7789_GMCTRN1 = ST7789_NVGAMCTRL = const(0xE1)

ST7789_PWCTR6 = const(0xFC)


ST77XX_MADCTL_MY = const(0x80)  # page address order (0: top to bottom; 1: bottom to top)
ST77XX_MADCTL_MX = const(0x40)  # column address order (0: left to right; 1: right to left)
ST77XX_MADCTL_MV = const(0x20)  # page/column order (0: normal mode 1; reverse mode)
ST77XX_MADCTL_ML = const(0x10)  # line address order (0: refresh to to bottom; 1: refresh bottom to top)
ST77XX_MADCTL_BGR = const(0x08) # colors are BGR (not RGB)
ST77XX_MADCTL_RTL = const(0x04) # refresh right to left
ST77XX_MADCTL_ROTS=(
    const(0x00),                                # 0 = portrait
    const(ST77XX_MADCTL_MX | ST77XX_MADCTL_MV), # 1 = landscape
    const(ST77XX_MADCTL_MY | ST77XX_MADCTL_MX), # 2 = inverted portrait
    const(ST77XX_MADCTL_MY | ST77XX_MADCTL_MV), # 3 = inverted landscape
)

ST77XX_COLOR_MODE_65K = const(0x50)
ST77XX_COLOR_MODE_262K = const(0x60)
ST77XX_COLOR_MODE_12BIT = const(0x03)
ST77XX_COLOR_MODE_16BIT = const(0x05)
ST77XX_COLOR_MODE_18BIT = const(0x06)
ST77XX_COLOR_MODE_16M = const(0x07)

# This is Pimoroni driver, with Adafruit header:
# https://github.com/pimoroni/st7789-python/blob/master/library/ST7789/__init__.py
# This is c++ Adafruit driver:
# https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp
# independent (?) micropython implementation:
# https://techatronic.com/st7789-display-pi-pico/

class St7789:
    def __init__(self, *, cs, dc, bl, spi, width=320, height=240, rot=1, dma=None):
        self.buf1 = bytearray(1)
        self.buf2 = bytearray(2)
        self.buf4 = bytearray(4)

        self.cs,self.dc,self.bl = [(machine.Pin(p,machine.Pin.OUT) if isinstance(p,int) else p) for p in (cs,dc,bl)]
        self.width = (width if rot%2 else height)
        self.height = (height if rot%2 else width)
        self.rot = rot

        self.dma = dma
        self.spi = spi
        self.config()

    def write_register(self, reg, buf=None):
        struct.pack_into('B', self.buf1, 0, reg)
        self.cs.value(0)
        self.dc.value(0)
        self.spi.write(self.buf1)
        if buf is not None:
            self.dc.value(1)
            self.spi.write(buf)
        self.cs.value(1)

    def write_register_dma(self, reg, buf, is_blocking=True ):
        'If *is_blocking* is False, used should call wait_dma explicitly.'
        SPI1_BASE = 0x40040000 # FIXME: will be different for another SPI bus?
        SSPDR     = 0x008
        self.dma.config(
            src_addr = uctypes.addressof(buf),
            dst_addr = SPI1_BASE + SSPDR,
            count    = len(buf),
            src_inc  = True,
            dst_inc  = False,
            trig_dreq= self.dma.DREQ_SPI1_TX
        )
        struct.pack_into('B',self.buf1,0,reg)
        self.cs.value(0)

        self.dc.value(0)
        self.spi.write(self.buf1)

        self.dc.value(1)
        self.dma.enable()

        if is_blocking: self.wait_dma()

    def wait_dma(self):
        while self.dma.is_busy(): pass
        self.dma.disable()
        # wait to send last byte. It should take < 1uS @ 10MHz 
        time.sleep_us(1)
        self.cs.value(1)

    def config(self):
        init7789=[
            # out of sleep mode
            (ST7789_SLPOUT, None, 100),     
            # memory access direction
            (ST7789_MADCTL, bytes([ST77XX_MADCTL_ROTS[self.rot%4]]), 0),    
            # RGB565
            (ST7789_COLMOD, bytes([ST77XX_COLOR_MODE_65K | ST77XX_COLOR_MODE_16BIT]), 0 ),
            # front/back porch setting in normal/idle/partial modes; 3rd byte (PSEN) 0x00 = disabled
            (ST7789_PORCTRL, b"\x0C\x0C\x00\x33\x33", 0), 
            # VGH=14.06V, VGL=-8.87V [Adafruit: 0x14]
            (ST7789_GCTRL, b"\x35", 0),
            # [Adafruit: missing]
            (ST7789_DISPOFF, b"\x28", 0),
            # power control [Adafruit: 0x2c]
            (ST7789_LCMCTRL, b"\x3C", 0),
            # power control (set VDV and VRD by register write), write VRH and VDV 
            (ST7789_VDVVRHEN, b"\x01", 0),(ST7789_VRHS, b"\x0B", 0),(ST7789_VDVS, b"\x20", 0),      
            # frame rate 60Hz
            (ST7789_FRCTRL2, b"\x0F", 0),
            # power control: AVDD=6.6V, AVCL=-4.8V, VDS=2.4V
            (ST7789_PWCTRL1, b"\xA4\xA1", 0),
            # positive voltage gamma control
            (ST7789_PVGAMCTRL, b"\xD0\x01\x08\x0F\x11\x2A\x36\x55\x44\x3A\x0B\x06\x11\x20", 0),
            # negative voltage gamma control
            (ST7789_NVGAMCTRL, b"\xD0\x02\x07\x0A\x0B\x18\x34\x43\x4A\x2B\x1B\x1C\x22\x1F", 0), 
            # content adaptive brightness control and color enhancement: color enhancement on, high enhancement
            (ST7789_WRCACE, bytes([0b1011_0000]), 0),
            # display on
            (ST7789_DISPON, None,100),        
        ]
        for reg,data,delay in init7789:
            self.write_register(reg,data)
            if delay>0: time.sleep_ms(delay)
        self.bl.value(1)

    def set_window(self, x, y, w, h):
        struct.pack_into('>hh', self.buf4, 0, x, x+w-1)
        self.write_register(ST7789_CASET, self.buf4)
        struct.pack_into('>hh', self.buf4, 0, y, y+h-1)
        self.write_register(ST7789_RASET, self.buf4)

    def blit(self, x, y, w, h, buf, is_blocking=True):
        self.set_window(x, y, w, h)
        if self.dma: self.write_register_dma(ST7789_RAMWR, buf, is_blocking)
        else: self.write_register(ST7789_RAMWR, buf)

    def clear(self, color):
        bs=128 # write pixels in chunks; makes the fill much faster
        struct.pack_into('>h',self.buf2,0,color)
        buf=bs*bytes(self.buf2)
        npx=self.width*self.height
        self.set_window(0, 0, self.width, self.height)
        self.write_register(ST7789_RAMWR, None)
        self.cs.value(0)
        self.dc.value(1)    
        for _ in range(npx//bs): self.spi.write(buf)
        for _ in range(npx%bs): self.spi.write(self.buf2)
        self.cs.value(1)

if __name__=='__main__':
    
    LCD_RST_PIN=15 # [unused]
    LCD_DC_PIN=8 # jgpeiro: 14
    LCD_CS_PIN=9
    LCD_CLK_PIN=10
    LCD_BKL_PIN=13 #jgpeiro: 15
    LCD_MOSI_PIN=11
    LCD_MISO_PIN=12
        
    def build_square_buf(w, h, inner=[0x00,0x00]):
        top = b"\xFF\xFF"*w
        body=(b"\xFF\xFF" + bytes(inner)*(w-2) + b"\xFF\xFF")*(h-2)
        bot = b"\xFF\xFF"*w
        return top + body + bot
        
    def test_lcd():
        spi = machine.SPI(
            1, 
            baudrate=24_000_000, 
            polarity=0,
            phase=0,
            sck=machine.Pin(LCD_CLK_PIN,machine.Pin.OUT),
            mosi=machine.Pin(LCD_MOSI_PIN,machine.Pin.OUT),
            miso=machine.Pin(LCD_MISO_PIN,machine.Pin.IN)
        )
        dma=rp2_dma.DMA(0)
        #dma=None
        lcd = St7789(rot=1,spi=spi,dma=dma,cs=LCD_CS_PIN,dc=LCD_DC_PIN,bl=LCD_BKL_PIN)
        lcd.clear(0x0000)
        
        # 1/4 screen pixels square with white border red backgorund 
        w, h = 320//4, 240//4
        bmp = build_square_buf(w, h, [0x03,0x03])
        
        t0 = time.ticks_us()
        lcd.blit(60, 60, w, h, bmp)
        t1 = time.ticks_us()

        print("Maximum FPS @24MHz:", 24e6/(320*240*16)) # FPS = F/(W*H*BPP)
        print("Achieved FPS:", 1/(16*(t1-t0)*1e-6))       # Note: Test only draws 1/16 of the sreen area
        
        print( "Draw TSC calibration pattern")
        w, h = 15, 15
        bmp = build_square_buf(w, h, [0x00,0x00])
        lcd.blit(50, 50, w, h, bmp)
        lcd.blit(250, 50, w, h, bmp)
        lcd.blit(250, 200, w, h, bmp)
        lcd.blit(50, 200, w, h, bmp)
    test_lcd()
