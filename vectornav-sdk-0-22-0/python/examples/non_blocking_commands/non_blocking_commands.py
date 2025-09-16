# The MIT License (MIT)
# 
# VectorNav SDK (v0.22.0)
# Copyright (c) 2024 VectorNav Technologies, LLC
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import sys, time, random

from vectornav import Sensor, Registers
from vectornav import GenericCommand, KnownMagneticDisturbance


def main(argv):
    """
    This example demonstrates how to send commands without blocking, validating each command was received correctly
    with different types of commands. For more information on non-blocking commands, please refer to the 'Advanced Functionality' 
    section of documentation/Documentation.html.

    This example will achieve the following:
    1. Connect to the sensor
    2. Configure the ADOR and ADOF to YPR at 100Hz
    3. Send generic command (known magnetic disturbance)
    4. Wait and check response
    5. Enter a loop for 5 seconds where it outputs VNYPR at 100 Hz and sends velocity aiding commands at 10 Hz:
        5.1. Check if a valid response has been received from the velocity aiding command
        5.2. Print response and send new command
    6. Disconnect from the sensor'
    """

    #### 1. AUTOCONNECT TO THE SENSOR

    sensorPortName = argv[0] if argv else "COM14" # Change the sensor port name to the com port of your local machine
    vs = Sensor()
    vs.autoConnect(sensorPortName)
    vs.verifySensorConnectivity()
    print(f"Connected to {sensorPortName} at {vs.connectedBaudRate()}")

    #### 2. CONFIGURE ADOR (ASYNCHRONOUS DATA OUTPUT REGISTER) AND ADOF (ASYNCHRONOUS DATA OUTPUT FREQUENCY)
    asyncDataOutputType = Registers.AsyncOutputType()
    asyncDataOutputType.ador = asyncDataOutputType.Ador.YPR # Configure for Yaw, Pitch, Roll data.
    asyncDataOutputType.serialPort = asyncDataOutputType.SerialPort.Serial1
    vs.writeRegister(asyncDataOutputType)
    print("ADOR configured")

    asyncDataOutputFreq= Registers.AsyncOutputFreq()
    asyncDataOutputFreq.adof = Registers.AsyncOutputFreq.Adof.Rate100Hz
    asyncDataOutputFreq.serialPort = Registers.AsyncOutputFreq.SerialPort.Serial1
    vs.writeRegister(asyncDataOutputFreq)
    print("ADOF Configured")

    #### 3. SEND KNOWN MAGNETIC DISTURBANCE COMMAND
    kmd = KnownMagneticDisturbance(KnownMagneticDisturbance.State.Present)
    vs.sendCommand(kmd, Sensor.SendCommandBlockMode.none) # Non-blocking

    #### 4. WAIT AND CHECK RESPONSE
    time.sleep(.25)

    # We could check kmd.awaitingResponse() but it will be removed from the command queue (setting kmd.awaitingResponse to false) after commandRemovalTimeoutLength (default 200ms), if any command has been sent or received since.
    if kmd.hasValidResponse():
        print(f"\nKMD Response: {kmd.getResponse()}")
        error_maybe = kmd.getError()
        if error_maybe is not None:
            print(f"Error: {error_maybe}")
    else:
        print(f"Error: KMD did not receive a valid response.")

    #### 5. ENTER LOOP FOR 5 SECONDS TO PROCESS COMMANDS AND MEASUREMENTS

    # Initialize velocity aiding register, command, counters, flags, and timers
    velAidRegister = Registers.VelAidingMeas()
    velAidWRGCommand = GenericCommand()

    velAidSentCount = 0
    asciiCount = 0
    validResponseReceived  = False

    start_time = time.time()
    resend_time = time.time()

    while time.time() - start_time < 5:
        # Handle ASCII YPR measurement
        compositeData = vs.getNextMeasurement(True)
        if compositeData and compositeData.matchesMessage("VNYPR"):
            ypr = compositeData.attitude.ypr
            print(f"YPR: {ypr.yaw}, {ypr.pitch}, {ypr.roll}")
            asciiCount += 1

        #### 5.1. CHECK FOR VALID RESPONSE
        if not validResponseReceived and not velAidWRGCommand.isAwaitingResponse():
            error_maybe = velAidWRGCommand.getError()
            if velAidWRGCommand.hasValidResponse():
                print(f"\nWRG Response: {velAidWRGCommand.getResponse()}")
                validResponseReceived = True
            elif error_maybe is not None:
                print(f"Error: {error_maybe}")

        #### 5.2. SEND COMMAND AND CHECK RESPONSE AT 2 HZ
        if (time.time() - resend_time) > 0.5:
            if not validResponseReceived and velAidSentCount > 0:
                print(f"Error: Response Timeout")
            
            velAidRegister.velocityX = random.random() # random.random() to simulate different velocities
            velAidRegister.velocityY = random.random()
            velAidRegister.velocityZ = random.random()
            velAidWRGCommand = velAidRegister.toWriteCommand()
            vs.sendCommand(velAidWRGCommand, Sensor.SendCommandBlockMode.none) # Non-blocking

            validResponseReceived = False
            velAidSentCount += 1
            resend_time = time.time() # Restart send timer

    print(f"\nTotal ASCII YPR Packets Received: {asciiCount}")
    print(f"Total Velocity Aiding Commands Sent: {velAidSentCount}")
    print("\nnonBlockingCommands example complete")

    #### 6. DISCONNECT THE SENSOR
    vs.disconnect()

if __name__ == '__main__':
    main(sys.argv[1:])
