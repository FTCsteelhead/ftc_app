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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Test: Auto Class", group = "Test")
//@Disabled
public class AutoClassTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();
        robot.init(hardwareMap);
        AutoRobotFunctions autoRobotFunctions = new AutoRobotFunctions((byte) 1, hardwareMap, this, robot);
       // autoRobotFunctions.setNavXPIDDriveStraight(0.33, 0.0008, 0.95);
      //  autoRobotFunctions.setNavXPIDTurn(0.33, 0.0008, 0.95);
        autoRobotFunctions.setGyroPID(.33, .0008, .95);
        robot.robotForward();

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();

        // make sure the gyro is calibrated.
     /*  while (!isStopRequested() && robot.gyro.isCalibrating()) {
            sleep(50);
            idle();
        }*/

        robot.gyro.getConnectionInfo();
        robot.gyro.getDeviceName();
        robot.gyro.resetZAxisIntegrator();
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.addData("Gyro", robot.gyro.getIntegratedZValue());
        telemetry.update();

        waitForStart();
        robot.setPoliceLED(true);
        //autoRobotFunctions.navxRotateToDegree(90, 2, -0.15, 0.15);
       // autoRobotFunctions.navXDriveStraight(0.0, 2, -0.15, 0.65, 0.5, 5000, 0.0005, 0.1,
              //  AutoRobotFunctions.StopConditions.BUTTON, 0);

      //  autoRobotFunctions.runWithEncoders(500, 0.5);

        autoRobotFunctions.MRRotate(90, 2, -.15, .15);
       // autoRobotFunctions.MRDriveStraight(90, .5, -.15,.75, 2, AutoRobotFunctions.StopConditions.BUTTON, 0);

     //   robot.gyro.getIntegratedZValue();
        robot.setPoliceLED(false);

        autoRobotFunctions.close();

        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}
