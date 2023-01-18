####################################################################
#                         PROGRAM - MASTER
# ATTENTION! Slave's program has to run at first!
#___________________________________________________________________

from machine import Pin, ADC, PWM, SoftI2C
from time import sleep, sleep_ms
import network
import espnow
import _thread
import ubinascii
from sh1106 import SH1106_I2C       # from sh1106.py
#import ssd1306                      # from ssd1306.py
import font10                       # from font10.py
from writer_minimal import Writer   # from writer_minimal.py

#-------------------------------------------------------------------
#===========================   SETUP   =============================
#-------------------------------------------------------------------

# input BUTTON pin definitions
POWER_BUTTON_PIN = Pin(25, Pin.IN, Pin.PULL_UP)  # button to turn on whole amplifier set
LCD_BUTTON_PIN = Pin(26, Pin.IN, Pin.PULL_UP)    # button to turn off LCD display
FREEZE_BUTTON_PIN = Pin(14, Pin.IN, Pin.PULL_UP) # button to turn off steering at all and freeze actual positions

# input BUTTON variable definitions as turned off
POWER_OFF = 1
LCD_OFF = 0
FREEZE_OFF = 1

# definitions of variables to check button press
pwr_first = 1
pwr_second = 1
lcd_first = 1
lcd_second = 1
frz_first = 1
frz_second = 1

# ADC pins definitions
ADC_bb = ADC(Pin(34))  # ADC for bass boost regulation measure
ADC_lp = ADC(Pin(39))  # ADC for low pass filter regulation measure
ADC_li = ADC(Pin(36))  # ADC for input level regulation measure

# ADC setup for full range: 3.3v
ADC_bb.atten(ADC.ATTN_11DB)
ADC_lp.atten(ADC.ATTN_11DB)
ADC_li.atten(ADC.ATTN_11DB)

# saving starting setup of potentiometers
ADC_bb_value = ADC_bb.read()
ADC_lp_value = ADC_lp.read()
ADC_li_value = ADC_li.read()

# global variables to display - also used by second thread
temp = 0        # temperature
vol_in = 0      # input voltage
cur_in = 0     # input current
pwr_out = 0     # output power
effic = 0       # efficiency

# other variables not used by second thread
vol_out = 0
cur_out = 0

#-------------------------------------------------------------------
#====================   FUNCTION DEFINITIONS   =====================
#-------------------------------------------------------------------

# function of second thread - to display data without making comunication slower
def second_thread_func():
    global POWER_OFF
    global LCD_OFF
    global FREEZE_OFF
    global temp
    global vol_in
    global cur_in
    global pwr_out
    global effic

    try:
        # I2C OLED display setup
        i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400000) 
        oled = SH1106_I2C(128, 64, i2c, None, addr=0x3C)
        font_writer = Writer(oled, font10)
        oled.fill(0)

        while True:
            if LCD_OFF:
                oled.fill(0)
                oled.show()
                sleep_ms(1000)

            else:
                oled.fill(0)

                # simplest way to calculate, but not most accurate
                pwr_out = vol_out * cur_out
                pwr_in = vol_in*cur_in
                if(pwr_in): 
                    effic = pwr_out/(pwr_in)*100
                else:
                    effic = 101

                # display temperature
                if temp < 100:
                    temp_str = str(round(temp, 2))
                elif temp < 300:
                    temp_str = str(round(temp, 1))
                else:
                    temp_str = 'ERR'
                font_writer.set_textpos(0, 0)
                font_writer.printstring(f'T={temp_str}')

                # display power_button and freeze mode
                if FREEZE_OFF:
                    font_writer.set_textpos(0, 72)
                    if POWER_OFF:
                        font_writer.printstring('a.off')
                    else:
                        font_writer.printstring('a.ON')
                else:
                    if POWER_OFF:
                        font_writer.set_textpos(3, 72)
                        font_writer.printstring('a.off')
                        font_writer.set_textpos(3, 113)
                        font_writer.printstring('fr')
                        oled.line(68, 0, 68, 18, 255)
                        oled.line(125, 0, 125, 18, 255)
                        oled.line(68, 18, 125, 18, 255)
                        oled.line(68, 0, 125, 0, 255)
                    else:
                        font_writer.set_textpos(3, 72)
                        font_writer.printstring('a.ON')
                        font_writer.set_textpos(3, 113)
                        font_writer.printstring('fr')
                        oled.line(68, 0, 68, 18, 255)
                        oled.line(125, 0, 125, 18, 255)
                        oled.line(68, 18, 125, 18, 255)
                        oled.line(68, 0, 125, 0, 255)

                # display input voltage
                if vol_in < 25:
                    vol_in_str = str(round(vol_in, 2))
                else:
                    vol_in_str = 'ERR'

                font_writer.set_textpos(22, 0)
                font_writer.printstring(f'u={vol_in_str}')

                # display input current
                if cur_in < 100:
                    cur_in_str = str(round(cur_in, 2))
                else:
                    cur_in_str = 'ERR'
                font_writer.set_textpos(22, 72)
                font_writer.printstring(f'i={cur_in_str}')

                # display output power
                if pwr_out < 1000:
                    pwr_out_str = str(round(pwr_out, 1))
                elif pwr_out < 10000:
                    pwr_out_str = str(round(pwr_out, 0))
                else:
                    pwr_out_str = 'ERR'
                font_writer.set_textpos(44, 0)
                font_writer.printstring(f'p={pwr_out_str}')

                # display efficiency
                if effic < 100:
                    effic_str = str(round(effic, 1))
                else:
                    effic_str = 'ERR'
                font_writer.set_textpos(44, 72)
                font_writer.printstring(f'n={effic_str}')
                
                oled.show() 
                sleep_ms(200)
    
    except (KeyboardInterrupt, Exception) as e:
        oled.text("caught exception {} {}".format(type(e).__name__, e), 0, 0)
        if exit_thread_sig:
            _thread.exit()

        
#-------------------------------------------------------------------
##########################   MAIN LOOP   ###########################
#-------------------------------------------------------------------
# UWAGA! Potrzebne jest załagodzenie odpowiedzi serwa na wachania napięcia
# odczytanego przez ADC - możliwe, że dobrym rozwiązaniem będzie wydajniejsze
# źródło zasilania oraz REGULATOR TRÓJSTAWNY
##### NA KONIEC, JEŚLI BĘDZIE DOBRY CZAS REAKCJI - WPROWADZIĆ MULTISAMPLING
 
# ATTENTION! Master's program has to run at least 200 ms later than slave
# more ms set for safety
sleep_ms(300)

# ESP-Now setup
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(mac = b'\xaa\xaa\xaa\xaa\xaa\xab')
e = espnow.ESPNow()
e.init()
peer = b'\xaa\xaa\xaa\xaa\xaa\xaa'   # MAC address of peer's wifi interface
e.add_peer(peer)

# starting second thread
exit_thread_sig = False
_thread.start_new_thread(second_thread_func, ())

on_duty = True

while on_duty:
    ### DO ZREALIZOWANIA W OBRĘBIE JEDNEJ ITERACJI:
    # 1. pomiar wartości z adc i przycisków
    # 2. przetworzenie ADC na nastawy serw
    # 3. wysłanie nastaw do slave
    # 4. odebranie danych diagnostycznych
    # 5. przerobienie i wyświetlenie info na LCD 

    pwr_first = POWER_BUTTON_PIN.value()
    lcd_first = LCD_BUTTON_PIN.value()
    frz_first = FREEZE_BUTTON_PIN.value()

    # 1. ADC MEASURE
    if FREEZE_OFF:
        ADC_bb_value = ADC_bb.read()
        ADC_lp_value = ADC_lp.read()
        ADC_li_value = ADC_li.read()

    # 2. ADC DATA PROCESSING
    bb_duty = int(30 + ADC_bb_value/4095*100)   # duty from 30 to 130
    lp_duty = int(30 + ADC_lp_value/4095*100)   # corresponding to servo
    li_duty = int(30 + ADC_li_value/4095*100)   # angle 0-180 deg

    # PREPARING DATA TO SEND
    controll_sum = bb_duty + lp_duty + li_duty + int(not POWER_OFF)
    msg_string = str(controll_sum) +'&'+ str(int(not POWER_OFF)) +'&'+ str(bb_duty) +'&'+ str(lp_duty) +'&'+ str(li_duty)

    # 3. SENDING MASTER'S DATA
    e.send(peer, msg_string, True)

    # 4. RECEIVING SLAVE'S DATA    
    host, msg = e.irecv()
    if msg:             # msg == None if timeout in irecv()
        # processing received data
        msg_string = msg.decode("utf-8")
        rcv_list = msg_string.split("&")

        # received data validation
        if(len(rcv_list) < 6): continue    # the message shouldn't contain less than 6 elements:
                                           # controll_sum, vol_in, vol_out, cur_in, cur_out, temp
        controll_sum = 0
        for i in range(1, len(rcv_list), 1):
            controll_sum += float(rcv_list[i])

        if((controll_sum - float(rcv_list[0]) >= 1) or controll_sum - float(rcv_list[0]) <= -1):
            continue   # comparing calculated sum with received controll sum

        # setting received variables
        vol_in  = float(rcv_list[1])
        vol_out = float(rcv_list[2])
        cur_in  = float(rcv_list[3])
        cur_out = float(rcv_list[4])
        temp    = float(rcv_list[5])

    sleep_ms(1)

    # checking the state of button pins and reacting
    pwr_second = POWER_BUTTON_PIN.value()
    lcd_second = LCD_BUTTON_PIN.value()
    frz_second = FREEZE_BUTTON_PIN.value()    
    
    if(pwr_first and not pwr_second):
        POWER_OFF = not POWER_OFF
    if(lcd_first and not lcd_second):
        LCD_OFF = not LCD_OFF
    if(frz_first and not frz_second):
        FREEZE_OFF = not FREEZE_OFF 

# after program ends    
e.send("end")
