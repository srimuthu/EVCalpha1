import msvcrt
import sys
import ArduinoComm

comm = ArduinoComm.ArduinoComm("COM5")
l_pwm = 0
r_pwm = 0
step = 3
init_val = 200
while True:
    pressedKey = msvcrt.getch()
    if pressedKey == 'w':
       print "w"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("FORWARD",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'a':
       print "a"
       l_pwm = l_pwm - step
       comm.SendMessage("ARC_TURN",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 's':
       print "s"
       l_pwm = init_val
       r_pwm = init_val
       comm.SendMessage("REVERSE",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'd':
       print "d"
       r_pwm = r_pwm - step
       comm.SendMessage("ARC_TURN",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'x':
       print "x"
       l_pwm = 0
       r_pwm = 0
       comm.SendMessage("STOP",[l_pwm,r_pwm,1,0])
       print l_pwm,r_pwm
    elif pressedKey == 'q':
       sys.exit()
    else:
       print "Key Pressed:" + str(pressedKey)