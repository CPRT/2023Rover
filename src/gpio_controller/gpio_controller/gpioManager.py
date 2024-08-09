import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

import Jetson.GPIO as GPIO
import interfaces.msg as GPIOmsg
import std_msgs.msg as Bool

class gpioManager(Node):
    def __init__(self):
        super().__init__("gpioNode")
        self.lastTimestamp = 0
        # mode = GPIO.getmode()
        self.gpiooutput = False
        GPIO.setmode(GPIO.BOARD)
        output_pins = {
            'JETSON_XAVIER': 18,
            'JETSON_NANO': 33,
            'JETSON_NX': 33,
            'CLARA_AGX_XAVIER': 18,
            'JETSON_TX2_NX': 32,
            'JETSON_ORIN': 18,
            'JETSON_ORIN_NX': 33,
            'JETSON_ORIN_NANO': 33
        }
        output_pin = output_pins.get(GPIO.model, None)
        if output_pin is None:
            raise Exception('PWM not supported on this board')
        self.cmd_move_subscriber = self.create_subscription(
             Bool,"/lights_on", self.lightCallback, 10) 
        # GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        # p = GPIO.PWM(output_pin, 50)
        # val = 25
        # incr = 5
        # p.start(val)

        # print("PWM running. Press CTRL+C to exit.")
        # try:
        #     while True:
        #         Sleep(0.25)
        #         if val >= 100:
        #             incr = -incr
        #         if val <= 0:
        #             incr = -incr
        #         val += incr
        #         p.ChangeDutyCycle(val)
        # finally:
        #     p.stop()
        #     GPIO.cleanup()

        self.lightRelay = [13]
        GPIO.setup(self.lightRelay, GPIO.OUT)
        # GPIO.output(lightRelay, GPIO.HIGH)
        freq = 1
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.ledTimer)

    def ledTimer(self):
        if(self.gpiooutput):
            GPIO.output(self.lightRelay, GPIO.HIGH)
            self.gpiooutput = False
        else:
            GPIO.output(self.lightRelay, GPIO.LOW)
            self.gpiooutput = True
    def lightCallback(self, msg: Bool):
        if(msg.data == True):
            GPIO.output(self.lightRelay, GPIO.HIGH)
        elif(msg.data == False):
            GPIO.output(self.lightRelay, GPIO.LOW)





def main(args=None):
    rclpy.init(args=args)
    node = gpioManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
