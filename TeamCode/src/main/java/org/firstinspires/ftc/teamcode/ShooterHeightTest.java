/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.HardwareSteelheadMainBot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StreamCorruptedException;
import java.util.Locale;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Test: Shooter Test", group = "Test")

public class ShooterHeightTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private File pathToCSV = null;
    private BufferedWriter writer = null;
    private long prevTime = 0;

    private VoltageSensor batVolt = null;
    private HardwareSteelheadMainBot robot = null;

    @Override
    public void init() {
        batVolt = hardwareMap.voltageSensor.iterator().next();
        robot = new HardwareSteelheadMainBot();
        robot.init(hardwareMap);

        try {
            pathToCSV = openFile("test.csv");

            writer = new BufferedWriter(new FileWriter(pathToCSV));
            writer.write("Time(min),Bat Voltage(volt),Height");
        } catch (IOException e) {
            telemetry.addData(">", "Write failed");
        }
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        robot.shooterPower(1);
        robot.setPoliceLED(true);
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        try {
            if ((runtime.seconds()-prevTime) >= 60) {
                prevTime = (long)runtime.seconds();
                writer.write("\n" + String.format(Locale.US, "%f", runtime.seconds()/60) + "," +
                        String.format(Locale.US,"%2.2f", batVolt.getVoltage()) + ",");
                robot.shooterServoDown(false);
                Thread.sleep(800);
                robot.shooterServoDown(true);
            }else if (batVolt.getVoltage() < 10 || robot.touchSensor.isPressed()) {
                if (writer != null)
                    writer.close();
                robot.shooterPower(0);
                robot.setPoliceLED(false);
                requestOpModeStop();
            }
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("PrevTime", prevTime);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    private File openFile(String fileName) throws IOException {
        File file = null;
        if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            telemetry.addData(">", "Able to mount Media");
            file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), fileName);
            telemetry.addData(">", file.toString());
            if (!file.exists()) {
                if (!file.createNewFile()) {
                    telemetry.addData("!", "Could not create file");
                }
            } else {
                telemetry.addData(">", "Directory exists");
            }
        }
        return file;
    }
}
