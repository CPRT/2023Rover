# Made by Alex De Meo

# Most of this code is sourced from randomnerdtutorials.com/raspberry-pi-ds18b20-python/
# At least as od Nov 22 2023, might soon be changed from that 
import os
import glob
import time


# Gets the temp readings from the w1_slave file
def read_raw_temp()->list[str]:
    file = open(device_file, 'r')
    lines = file.readlines()
    file.close()
    return lines


def read_temp()->float:
    # Getting the temp readings
    lines = read_raw_temp()

    # Basically just checking to see if the data is valid
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_raw_temp()
        equals_pos = lines[1].find('t=')
    
    if equals_pos != -1:
        temp_string = lines[1][equals_pos + 2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c
    
    

def main():
    # Loading the kernel modules
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')

    # the file directory where one wire devices are located
    # !!!!---IMPORTANT---!!!! jetson might not actually have the w1 folder,
    # Replace this file directory when the jetson is prepped
    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + "28*")[0]
    # Again check the w1 thing
    device_file = device_folder + '/w1_slave'


if __name__ == '__main__':
    main()
