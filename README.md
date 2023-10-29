# esp_obichase
# A chase game for dogs 

## Description
 DC Motor speed controller useing RC (Radio Control) transmitter/receiver 
 allowing fine contol forward and backwards.
  
 Utilises a Transmitter switch to Arm or Disarm (stop) the motor.
 To Arm (allow motor to run) the Arm switch must be set and speed zero. This is
 a saftey feature to prevent the motor spiining up unintentially.
  
 Setting the saftey switch while the motor is running will force motor to stop.
  
### Uses:
- LCD 1602
- MDC30 R2: Motor Speed Controller
- AndyMark, am-0255 DC Motor 
- ESP32
- RC receiver using two channels:
    - a channel to control direction and speed -255 to + 255
    - a channel to contol saftey  - arm or disarm


