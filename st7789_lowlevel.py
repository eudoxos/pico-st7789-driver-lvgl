import time
import machine
import struct
import uctypes

from micropython import const
import rp2_dma

ST77XX_NOP = const(0x00)
ST77XX_SWRESET = const(0x01)
ST77XX_RDDID = const(0x04)
ST77XX_RDDST = const(0x09)

ST77XX_SLPIN = const(0x10)
ST77XX_SLPOUT = const(0x11)
ST77XX_PTLON = const(0x12)
ST77XX_NORON = const(0x13)

ST77XX_INVOFF = const(0x20)
ST77XX_INVON = const(0x21)
ST77XX_DISPOFF = const(0x28)
ST77XX_DISPON = const(0x29)

ST77XX_CASET = const(0x2A)
ST77XX_RASET = const(0x2B)
ST77XX_RAMWR = const(0x2C)
ST77XX_RAMRD = const(0x2E)

ST77XX_PTLAR = const(0x30)
ST77XX_MADCTL = const(0x36)
ST77XX_COLMOD = const(0x3A)

ST7789_WRCACE = const(0x55)

ST77XX_FRMCTR1 = const(0xB1)
ST77XX_FRMCTR2 = ST7789_PORCTRL = const(0xB2)
ST77XX_FRMCTR3 = const(0xB3)
ST77XX_INVCTR = const(0xB4)
ST7789_DISSET5 = const(0xB6)

ST7789_GCTRL = const(0xB7)
ST7789_GTADJ = const(0xB8)
ST7789_VCOMS = const(0xBB)

ST7735_PWCTR1 = ST7789_LCMCTRL = const(0xC0)
ST7735_PWCTR2 = ST7789_IDSET = const(0xC1)
ST7735_PWCTR3 = ST7789_VDVVRHEN = const(0xC2)
ST7735_PWCTR4 = ST7789_VRHS = const(0xC3)
ST7735_PWCTR5 = ST7789_VDVS = const(0xC4)
ST7735_VMCTR1 = ST7789_VMCTR1 = const(0xC5)
ST7789_FRCTRL2 = const(0xC6)
ST7789_CABCCTRL = const(0xC7)

ST7789_PWCTRL1 = const(0xD0)
ST77XX_RDID1 = const(0xDA)
ST77XX_RDID2 = const(0xDB)
ST77XX_RDID3 = const(0xDC)
ST77XX_RDID4 = const(0xDD)

ST7789_GMCTRP1 = ST7789_PVGAMCTRL = const(0xE0)
ST7789_GMCTRN1 = ST7789_NVGAMCTRL = const(0xE1)

ST7735_PWCTR6 = ST7789_PWCTR6 = const(0xFC)


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
# st77xx c driver (for uPy), with simplified init sequence:
# https://github.com/szampardi/st77xx_mpy

class St77xx(object):
    def __init__(self, *, cs, dc, bl, spi, rst=None, res=(320,240), rot=1, dma=None):
        '''
        * *cs*: chip select pin (= slave select, SS)
        * *dc*: data/command pin
        * *bl*: backlight PWM pin
        * *rst*: optional reset pin
        * *res*: resolution tuple; greater value always comes first
        * *rot*: display orientation (0: landscape, 1: portrait, 2: reverse landscape, 3: reverse portrait)
        '''
        self.buf1 = bytearray(1)
        self.buf2 = bytearray(2)
        self.buf4 = bytearray(4)

        self.cs,self.dc,self.bl,self.rst = [(machine.Pin(p,machine.Pin.OUT) if isinstance(p,int) else p) for p in (cs,dc,bl,rst)]
        self.bl.value(1)
        self.bl=machine.PWM(self.bl)
        self.width = (res[0] if rot%2 else res[1])
        self.height = (res[1] if rot%2 else res[0])
        self.rot = rot

        self.dma = dma
        self.spi = spi
        self.config()

    def off(self):
        self.bl.value(0)
    def hard_reset(self):
        if self.rst:
            for v in (1,0,1):
                self.rst.value(v)
                time.sleep(.5)
            time.sleep(.5)
        self.config()
    def set_backlight(self,percent):
        self.bl.duty_u16(percent*655)
    def set_window(self, x, y, w, h):
        struct.pack_into('>hh', self.buf4, 0, x, x+w-1)
        self.write_register(ST77XX_CASET, self.buf4)
        struct.pack_into('>hh', self.buf4, 0, y, y+h-1)
        self.write_register(ST77XX_RASET, self.buf4)

    def blit(self, x, y, w, h, buf, is_blocking=True):
        self.set_window(x, y, w, h)
        if self.dma: self.write_register_dma(ST77XX_RAMWR, buf, is_blocking)
        else: self.write_register(ST77XX_RAMWR, buf)

    def clear(self, color):
        bs=128 # write pixels in chunks; makes the fill much faster
        struct.pack_into('>h',self.buf2,0,color)
        buf=bs*bytes(self.buf2)
        npx=self.width*self.height
        self.set_window(0, 0, self.width, self.height)
        self.write_register(ST77XX_RAMWR, None)
        self.cs.value(0)
        self.dc.value(1)    
        for _ in range(npx//bs): self.spi.write(buf)
        for _ in range(npx%bs): self.spi.write(self.buf2)
        self.cs.value(1)
        
    def write_register(self, reg, buf=None):
        struct.pack_into('B', self.buf1, 0, reg)
        self.cs.value(0)
        self.dc.value(0)
        self.spi.write(self.buf1)
        if buf is not None:
            self.dc.value(1)
            self.spi.write(buf)
        self.cs.value(1)

    def write_register_dma(self, reg, buf, is_blocking=True):
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

    def _run_seq(self,seq):
        '''
        Run sequence of (initialization) commands; those are given as list of tuples, which are either
        `(command,data)` or `(command,data,delay_ms)`
        '''
        for i,cmd in enumerate(seq):
            if len(cmd)==2: (reg,data),delay=cmd,0
            elif len(cmd)==3: reg,data,delay=cmd
            else: raise ValueError('Command #%d has %d items (must be 2 or 3)'%(i,len(cmd)))
            self.write_register(reg,data)
            if delay>0: time.sleep_ms(delay)

        
class St7735(St77xx):
    '''There are many variants of ST7735-based LCDs, none of them seem to be working yet'''
    def __init__(self,res,**kw):
        suppRes=[(160,128),]
        if res not in suppRes: raise ValueError('Unsupported resolution %s; the driver currently supports %s.'%(str(res,', '.join(str(r) for r in suppRes))))
        super().__init__(res=res,**kw)
    def config(self):
        # mostly from here
        # https://github.com/stechiez/raspberrypi-pico/blob/main/pico_st7735/st7735/ST7735.py
        # the "blue version" only
        init7735r=[
            # see here for explanations: https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp
            (ST77XX_SWRESET, None, 150),
            (ST77XX_SLPOUT, None, 255),
            (ST77XX_FRMCTR1,b'\x01\x2c\x2d'),
            (ST77XX_FRMCTR2,b'\x01\x2c\x2d'),
            (ST77XX_FRMCTR3,b'\x01\x2c\x2d\x01\x2c\x2d'),
            (ST77XX_INVCTR,b'\x07'),
            (ST7735_PWCTR1,b'\xa2\x02\xb4'),
            (ST7735_PWCTR2,b'\xc5'),
            (ST7735_PWCTR3,b'\x0a\x00'),
            (ST7735_PWCTR4,b'\x8a\x2a'),
            (ST7735_PWCTR5,b'\x8a\xee'),
            (ST7735_VMCTR1,b'\x0e'),
            (ST77XX_INVOFF,None),
            (ST77XX_MADCTL,bytes([ST77XX_MADCTL_ROTS[self.rot%4]])),    
            (ST77XX_COLMOD,bytes([ST77XX_COLOR_MODE_65K | ST77XX_COLOR_MODE_16BIT])),
            (ST77XX_CASET,bytes([0x00,0x00,0x00,0x7f])),
            (ST77XX_RASET,bytes([0x00,0x00,0x00,0x9f])),
            # blacktab only
            (ST77XX_MADCTL,b'\xc0'),
            # TODO: gamma adjustment
            (ST77XX_NORON, None, 10),
            (ST77XX_DISPON, None,100)
        ]
        init7735=[
            # swreset
            (ST77XX_SWRESET, None, 50),
            # out of sleep mode
            (ST77XX_SLPOUT, None, 100),
            # RGB565
            (ST77XX_COLMOD,bytes([ST77XX_COLOR_MODE_65K | ST77XX_COLOR_MODE_16BIT])),
            # fast refresh (??)
            (ST77XX_FRMCTR1,bytes([0x00,0x06,0x03])),
            (ST77XX_MADCTL,bytes([0x03])),
            (ST77XX_INVCTR,b'\x00'),
            (ST7735_PWCTR1,b'\x02\x70'),
            (ST7735_PWCTR2,b'\x05'),
            (ST7735_PWCTR3,b'\x01\x02'),
            (ST7735_VMCTR1,b'\x3c\x38'),
            (ST7735_PWCTR6,b'\b11\b15'),
            # (ST77XX_GMCTRP1,b'\
            ## memory access direction
            #(ST77XX_MADCTL, bytes([ST77XX_MADCTL_ROTS[self.rot%4]]), 0),    
            # inverted on (?)
            #(ST77XX_INVON, None, 10),
            # normal display on
            (ST77XX_NORON, None, 10),
            # display on
            (ST77XX_DISPON, None,100)
        ]
        self._run_seq(init7735r)


class St7789(St77xx):
    def __init__(self,res,**kw):
        suppRes=[(320,240),]
        if res not in suppRes: raise ValueError('Unsupported resolution %s; the driver currently supports %s.'%(str(res,', '.join(str(r) for r in suppRes))))
        super().__init__(res=res,**kw)
    def config(self):
        init7789=[
            # out of sleep mode
            (ST77XX_SLPOUT, None, 100),     
            # memory access direction
            (ST77XX_MADCTL, bytes([ST77XX_MADCTL_ROTS[self.rot%4]])),    
            # RGB565
            (ST77XX_COLMOD, bytes([ST77XX_COLOR_MODE_65K | ST77XX_COLOR_MODE_16BIT])),
            # front/back porch setting in normal/idle/partial modes; 3rd byte (PSEN) 0x00 = disabled
            (ST7789_PORCTRL, b"\x0C\x0C\x00\x33\x33"), 
            # VGH=14.06V, VGL=-8.87V [Adafruit: 0x14]
            (ST7789_GCTRL, b"\x35"),
            # [Adafruit: missing]
            (ST77XX_DISPOFF, b"\x28"),
            # power control [Adafruit: 0x2c]
            (ST7789_LCMCTRL, b"\x3C"),
            # power control (set VDV and VRD by register write), write VRH and VDV 
            (ST7789_VDVVRHEN, b"\x01"),(ST7789_VRHS, b"\x0B"),(ST7789_VDVS, b"\x20"),      
            # frame rate 60Hz
            (ST7789_FRCTRL2, b"\x0F"),
            # power control: AVDD=6.6V, AVCL=-4.8V, VDS=2.4V
            (ST7789_PWCTRL1, b"\xA4\xA1"),
            # positive voltage gamma control
            (ST7789_PVGAMCTRL, b"\xD0\x01\x08\x0F\x11\x2A\x36\x55\x44\x3A\x0B\x06\x11\x20"),
            # negative voltage gamma control
            (ST7789_NVGAMCTRL, b"\xD0\x02\x07\x0A\x0B\x18\x34\x43\x4A\x2B\x1B\x1C\x22\x1F"),
            # content adaptive brightness control and color enhancement: color enhancement on, high enhancement
            (ST7789_WRCACE, bytes([0b1011_0000])),
            # display on
            (ST77XX_DISPON, None,100),        
        ]
        self._run_seq(init7789)


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

    def test_lcd(lcd):
        # lcd.hard_reset()
        lcd.set_backlight(30)

        lcd.clear(0x0000)    
        # 1/4 screen pixels square with white border red backgorund 
        w, h = lcd.width//4, lcd.height//4
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
        for p in (20,100,80,50,10,0):
            lcd.set_backlight(p)
            time.sleep(.5)

    spi = machine.SPI(
        1, 
        baudrate=20_000_000, 
        polarity=0,
        phase=0,
        sck=machine.Pin(LCD_CLK_PIN,machine.Pin.OUT),
        mosi=machine.Pin(LCD_MOSI_PIN,machine.Pin.OUT),
        miso=machine.Pin(LCD_MISO_PIN,machine.Pin.IN)
    )
    dma=rp2_dma.DMA(0)

    lcd7789=St7789(rot=1,res=(320,240),spi=spi,dma=dma,cs=LCD_CS_PIN,dc=LCD_DC_PIN,bl=LCD_BKL_PIN,rst=LCD_RST_PIN)
    test_lcd(lcd=lcd7789)
    #lcd7735=St7735(rot=1,res=(160,128),spi=spi,dma=None,rst=16,dc=17,cs=18,bl=19)
    #test_lcd(lcd=lcd7735)
