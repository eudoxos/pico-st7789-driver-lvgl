import sys
sys.path.append('.')
from st7789_lowlevel import *
from xpt2046 import *


spi=machine.SPI(
    1,
    baudrate=24_000_000, 
    polarity=0,
    phase=0,
    sck=machine.Pin(10,machine.Pin.OUT),
    mosi=machine.Pin(11,machine.Pin.OUT),
    miso=machine.Pin(12,machine.Pin.IN)
)

## TODO: measure fps gain w/ DMA
# dma=rp2_dma.DMA(0)
dma=None

lcd=St7789(rot=3,res=(240,320),spi=spi,dma=dma,cs=9,dc=8,bl=13,rst=15)
time.sleep(.1)
lcd=St7789(rot=3,res=(240,320),spi=spi,dma=dma,cs=9,dc=8,bl=13,rst=15)
lcd.set_backlight(30)
touch=Xpt2046(spi=spi,cs=16,rot=1)

import time
import machine
import lvgl as lv
import sys

print("lv.init()")
lv.init()
print("disp_buf_t.init()")
HRES,VRES=lcd.width,lcd.height
# set fb2 to None to disable double-buffering
fb1,fb2=bytearray(HRES*2*32),bytearray(HRES*2*32)
disp_draw_buf=lv.disp_draw_buf_t()
disp_draw_buf.init(fb1,fb2,len(fb1)//lv.color_t.__SIZE__)
if lv.COLOR.DEPTH!=16 or not lv.COLOR_16.SWAP: raise RuntimeError(f'LVGL *must* be compiled with 16bit color depth and swapped bytes (current: lv.COLOR.DEPTH={lv.COLOR.DEPTH}, lv.COLOR_16.SWAP={lv.COLOR_16.SWAP})')

is_fb1=True
def disp_drv_flush_cb(disp_drv,area,color):
    global is_fb1
    print(f"({area.x1},{area.y1}..{area.x2},{area.y2})")
    fb=memoryview(fb1 if (is_fb1 or fb2 is None) else fb2)
    is_fb1=not is_fb1
    lcd.wait_dma() # wait if not yet done
    # blit in background
    # FIXME: check that data have the correct size (LV_COLOR_DEPTH)
    lcd.blit(area.x1,area.y1,w:=(area.x2-area.x1+1),h:=(area.y2-area.y1+1),fb[0:2*w*h],is_blocking=True) # is_blocking=False)
    disp_drv.flush_ready()
    
print('driver')
disp_drv=lv.disp_drv_t()
disp_drv.init()

disp_drv.draw_buf=disp_draw_buf
disp_drv.flush_cb=disp_drv_flush_cb
disp_drv.hor_res=lcd.width
disp_drv.ver_res=lcd.height
disp_drv.register()

def indev_drv_read_cb(indev_drv, data):
    lcd.wait_dma()
    print('-',end='')
    spi.init(baudrate=1_000_000)
    pos=touch.pos()
    if pos is None: data.state=0
    else: (data.point.x,data.point.y),data.state=pos,1
    print('#',end='')
    spi.init(baudrate=24_000_000)
    return False


if 1:
    print("indev_drv_t.init()")
    indev_drv=lv.indev_drv_t()
    indev_drv.init()
    indev_drv.type=lv.INDEV_TYPE.POINTER
    indev_drv.read_cb=indev_drv_read_cb
    indev_drv.register()

def cb_btn(event): print("Hello World!")


print('lv.obj()')
scr=lv.obj()
print('... okay!')
btn=lv.btn(scr)
lbl=lv.label(btn)
lbl.set_text("Press me!")
btn.center()

btn.add_event_cb(cb_btn,lv.EVENT.CLICKED,None)
lv.scr_load(scr)


import lv_utils
import uasyncio
lv_utils.event_loop(refresh_cb=lv.task_handler,asynchronous=True)

uasyncio.Loop.run_forever()

