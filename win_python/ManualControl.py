import getch
import sys
import ArduinoComm

comm = ArduinoComm.ArduinoComm("COM4")
l_pwm = 0
r_pwm = 0
step = 3
init_val = 230
pan_init = 90
tilt_init = 90
pan = 90
tilt = 90
while True:
    pressedKey = getch.getch()
    if pressedKey == 'w':
       print "w"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("FORWARD",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'a':
       print "a"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("ARC_LEFT",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 's':
       print "s"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("REVERSE",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'd':
       print "d"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("ARC_RIGHT",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'x':
       print "x"
       l_pwm = 0
       r_pwm = 0
       comm.SendMessage("STOP",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'i':
       print "tilt up"
       tilt = tilt +1
       comm.SendMessage("TILT_SERVO",[tilt,0,0,0])
       print tilt
    elif pressedKey == 'k':
       print "tilt down"
       tilt = tilt -1
       comm.SendMessage("TILT_SERVO",[tilt,0,0,0])
       print tilt
    elif pressedKey == 'j':
       print "pan left"
       pan = pan +1
       comm.SendMessage("PAN_SERVO",[pan,0,0,0])
       print pan
    elif pressedKey == 'l':
       print "pan right"
       pan = pan -1
       comm.SendMessage("PAN_SERVO",[pan,0,0,0])
       print pan
    elif pressedKey == 'q':
       sys.exit()
    else:
       print "Key Pressed:" + str(pressedKey)
