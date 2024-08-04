import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

import Jetson.GPIO as GPIO
import interfaces.msg as GPIOmsg


class joystickDrive(Node):
    def __init__(self):
        super().__init__("gpioNode")
        self.lastTimestamp = 0
        mode = GPIO.getmode()
        GPIO.setmode(mode)
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
        
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        p = GPIO.PWM(output_pin, 50)
        val = 25
        incr = 5
        p.start(val)

        print("PWM running. Press CTRL+C to exit.")
        try:
            while True:
                Sleep(0.25)
                if val >= 100:
                    incr = -incr
                if val <= 0:
                    incr = -incr
                val += incr
                p.ChangeDutyCycle(val)
        finally:
            p.stop()
            GPIO.cleanup()

        channels = [18, 12, 13]
        GPIO.setup(channels, GPIO.OUT)
        GPIO.output(channels, 5)



def main(args=None):
    rclpy.init(args=args)
    node = joystickDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
