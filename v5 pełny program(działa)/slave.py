####################################################################
#                         PROGRAM - SLAVE
#___________________________________________________________________

from machine import Pin, ADC, PWM, SoftI2C
import onewire
import ds18x20
import time
# import os
# import math
# import struct
import network
import espnow
import ubinascii


#-------------------------------------------------------------------
#===========================   SETUP   =============================
#-------------------------------------------------------------------

# SERVO pins definitions
SERVO_BB_PIN = Pin(19, Pin.OUT)  # sevo bass boost
SERVO_LP_PIN = Pin(18, Pin.OUT)  # servo low pass
SERVO_IL_PIN = Pin(5, Pin.OUT)  # serwo input lvl

# ON/OFF coil/transmitter pin definition
ON_OFF_PIN = Pin(17, Pin.OUT)
#ON_OFF_PIN = Pin(2, Pin.OUT) ##############  ONLY FOR DEBUG  ###############

# ADC pins definitions
ADC_vol_in  = ADC(Pin(36))  # amplifier input (power) voltage measure
ADC_vol_out = ADC(Pin(39))  # amplifier output (phones) voltage measure
ADC_cur_in  = ADC(Pin(34))  # data from input current meter voltage measure
ADC_cur_out = ADC(Pin(35))  # data from output current meter voltage measure

# ADC setup for full range: 3.3v
ADC_vol_in.atten(ADC.ATTN_11DB)
ADC_vol_out.atten(ADC.ATTN_11DB)
ADC_cur_in.atten(ADC.ATTN_11DB)
ADC_cur_out.atten(ADC.ATTN_11DB)

# saving starting setup of potentiometers
ADC_vol_in_val = ADC_vol_in.read()
ADC_vol_out_val = ADC_vol_out.read()
ADC_cur_in_val = ADC_cur_in.read()
ADC_cur_out_val = ADC_cur_out.read()

# SERVO PWM definitions
frequency = 50    # 50 Hz
servo_bb = PWM(SERVO_BB_PIN, freq = frequency)
servo_lp = PWM(SERVO_LP_PIN, freq = frequency)
servo_il = PWM(SERVO_IL_PIN, freq = frequency)

# temperature sensor setup
ds_pin = Pin(23)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()
ds_serial_code = roms[0]


#-------------------------------------------------------------------
#===================   SETTING INITIAL VALUES  =====================
#-------------------------------------------------------------------

# SERVO pins setup
SERVO_BB_PIN.off()
SERVO_LP_PIN.off()
SERVO_IL_PIN.off()

# ON/OFF coil/transmitter pin setup
ON_OFF_PIN.off()

# extreme values for servo duty
DUTY_MAX = 130
DUTY_MIN = 30

#-------------------------------------------------------------------
##########################   MAIN LOOP   ###########################
#-------------------------------------------------------------------
# UWAGA! Potrzebne jest załagodzenie odpowiedzi serwa na wachania napięcia
# odczytanego przez ADC - możliwe, że dobrym rozwiązaniem będzie wydajniejsze
# źródło zasilania oraz REGULATOR TRÓJSTAWNY
##### NA KONIEC, JEŚLI BĘDZIE DOBRY CZAS REAKCJI - WPROWADZIĆ MULTISAMPLING

# activating a WLAN interface
w0 = network.WLAN(network.STA_IF)
w0.active(True)
w0.config(mac = b'\xaa\xaa\xaa\xaa\xaa\xaa')  # setting MAC adress of local device

e = espnow.ESPNow()
e.init()
peer = b'\xaa\xaa\xaa\xaa\xaa\xab'  # MAC address of peer's wifi interface
e.add_peer(peer)

on_duty = True

while on_duty:
    host, msg = e.irecv()
    if msg:             # msg == None if timeout in irecv()
        # processing received data
        msg_string = msg.decode("utf-8")
        rcv_list = msg_string.split("&")

        # received data validation
        if(len(rcv_list) < 5): continue    # the message shouldn't contain less than 5 elements:
                                           # controll_sum, power_on, bb_duty, lp_duty, li_duty 
        controll_sum = 0
        for i in range(1, len(rcv_list), 1):
            controll_sum += int(rcv_list[i])
        if(controll_sum != int(rcv_list[0])):
            continue   # comparing calculated sum with received controll sum

        # setting power coil
        # if((rcv_list[1] == '0') or (rcv_list[1] == '1')):
        #     ON_OFF_PIN.value(int(rcv_list[1]))
        ON_OFF_PIN.value(int(rcv_list[1]))
        print(rcv_list[1])

        # setting servo duties
        bb_duty = int(rcv_list[2])
        if((bb_duty >= DUTY_MIN) and (bb_duty <= DUTY_MAX)):
            servo_bb.duty(bb_duty)
        
        lp_duty = int(rcv_list[3])
        if((lp_duty >= DUTY_MIN) and (lp_duty <= DUTY_MAX)):
            servo_lp.duty(lp_duty)

        il_duty = int(rcv_list[4])
        if((il_duty >= DUTY_MIN) and (il_duty <= DUTY_MAX)):
            servo_il.duty(il_duty)

        # preparing temperature info to send
        ds_sensor.convert_temp()
        temp = round(ds_sensor.read_temp(ds_serial_code), 2)
        # temp_str = "{num:.2f}".format(num = temp)

        # preparing ADC data to send
        # ADC MEASURE
        ADC_vol_in_val  = ADC_vol_in.read()
        ADC_vol_out_val = ADC_vol_out.read()
        ADC_cur_in_val  = ADC_cur_in.read()
        ADC_cur_out_val = ADC_cur_out.read()

        # 2. ADC DATA PROCESSING
        # vol_in voltage divider paramethers: k=1/7,8, R1=68k, R2=10k, Vout=V_R2
        if (ADC_vol_in_val > 3000):
            vol_in = round((0.0005*ADC_vol_in_val + 1.0874)*7.8, 2)
        else:
            vol_in = round((0.0008*ADC_vol_in_val + 0.1372)*7.8, 2)

        # vol_out voltage divider paramethers: k=1/15,7058, R1=100k, R2=6k8, Vout=V_R2
        if (ADC_vol_out_val > 3000):
            vol_out = round((0.0005*ADC_vol_out_val + 1.0874)*15.7058, 2)
        else:
            vol_out = round((0.0008*ADC_vol_out_val + 0.1372)*15.7058, 2)
        
        # cur_in meter voltage: 2.5V = 50A
        if (ADC_cur_in_val > 3000):
            cur_in = round((0.0005*ADC_cur_in_val + 1.0874)*20, 2)
        else:
            cur_in = round((0.0008*ADC_cur_in_val + 0.1372)*20, 2)
        
        # cur_out meter voltage: 1V = 15A
        if (ADC_cur_out_val > 3000):
            cur_out = round((0.0005*ADC_cur_out_val + 1.0874)*15, 2)
        else:
            cur_out = round((0.0008*ADC_cur_out_val + 0.1372)*15, 2)

        # PREPARING DATA TO SEND
        controll_sum_send = vol_in + vol_out + cur_in + cur_out + temp
        string_to_send = str(controll_sum_send) +'&'+ str(vol_in) +'&'+ str(vol_out) +'&'+ str(cur_in) +'&'+ str(cur_out) +'&'+ str(temp)

        # SENDING SLAVE'S DATA
        sent = e.send(peer, string_to_send, True)
                
# ON/OFF coil/transmitter pin setup
ON_OFF_PIN.off()

# SERVO pins setup
SERVO_BB_PIN.off()
SERVO_LP_PIN.off()
SERVO_IL_PIN.off()

# cleanup
exit_thread_sig = True
time.sleep_ms(50)
print("Done")
_thread.exit()
