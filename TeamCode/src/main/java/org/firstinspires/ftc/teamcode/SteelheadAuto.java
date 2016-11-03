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

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.ColorPIDController;
import org.steelhead.ftc.HardwareSteelheadMainBot;

import java.text.DecimalFormat;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Auto", group = "Steelhead")
//@Disabled
public class SteelheadAuto extends LinearOpMode {


    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

    ColorPIDController pidController;

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navxDevice;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_UPDATE_RATE_HZ = 50;
    private final double TOLERANCE_DEGREES = 2.0;

    //Proportional, Integral & Derivative Values init
    private final double YAW_PID_P = 0.06;
   // private final double YAW_PID_P = 0.01;
    private final double YAW_PID_I = 0.0012;
    private final double YAW_PID_D = 0.85;

    private double MAX_OUTPUT_VAL = 0.25;
    private double MIN_OUTPUT_VAL = -0.25;

    private boolean calibrationComplete = false;
    private boolean rotateComplete = false;
   // private boolean lineReached = false;





    @Override
    public void runOpMode()throws InterruptedException{

    /* get the navx device from a device interface module named "dim"
     * Then create a new PID controller with the timestamped source YAW
     */
        robot.init(hardwareMap);

        robot.robotBackward();

        navxDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_UPDATE_RATE_HZ);

        yawPIDController = new navXPIDController(navxDevice,
                navXPIDController.navXTimestampedDataSource.YAW);

        //Configure the device
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);


        robot.robotSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!calibrationComplete) {
            calibrationComplete = !navxDevice.isCalibrating();
            telemetry.addData("CAL:", "Calibration in progress");
        }
        navxDevice.zeroYaw();

        telemetry.addData("STATUS:", "init complete");
        telemetry.update();

       // robot.color.enableLed(true);
      //  robot.color.enableLed(false);
      //  robot.color.enableLed(true);

       //  robot.runWithEncoders();


        navXPIDController.PIDResult PIDResult = new navXPIDController.PIDResult();
        int DEVICE_TIMEOUT_MS = 500;

        DecimalFormat df = new DecimalFormat("#.##");

      //  pidController = new ColorPIDController(robot.color, 3, 43);
        pidController.setPID(0.018, 0.05, 0.00203);


      waitForStart();

        robot.robotBackward();

           MIN_OUTPUT_VAL = -0.5;
           MAX_OUTPUT_VAL = 0.5;


           yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
           yawPIDController.enable(true);

           double driveSpeed = 0.25;

           //drives straight for 500 encoder counts
            while (robot.rightMotor_2.getCurrentPosition() < 500 || robot.leftMotor_2.getCurrentPosition() < 500
                    && !Thread.currentThread().isInterrupted() && opModeIsActive()) {

                telemetry.addData("right encoder count", robot.rightMotor_2.getCurrentPosition());
                telemetry.addData("left encoder count", robot.leftMotor_2.getCurrentPosition());
                telemetry.update();

                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.robotLeftPower(driveSpeed);
                        robot.robotRightPower(driveSpeed);
                        telemetry.addData("PID OUT:", df.format(0.00));
                    } else {
                        double output = PIDResult.getOutput();
                        robot.leftMotor_1.setPower(limit(driveSpeed + output));
                        robot.leftMotor_2.setPower(limit(driveSpeed + output));
                        robot.rightMotor_1.setPower(limit(driveSpeed - output));
                        robot.rightMotor_2.setPower(limit(driveSpeed - output));
                        telemetry.addData("MOTOR OUT:", df.format(limit(driveSpeed + output))
                                + " , " + df.format(limit(driveSpeed - output)));
                    }

                } else {
                    Log.w("navx", "YAW PID TIMEOUT");
                }
            }
                yawPIDController.enable(false);
              //  robot.runWithoutEncoders();

                yawPIDController.setSetpoint(45.0);

                yawPIDController.enable(true);

            //turns 45 degrees
            while (!rotateComplete && !Thread.currentThread().isInterrupted() && opModeIsActive()) {
                telemetry.addData("Yaw", navxDevice.getYaw());
                telemetry.update();
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.robotLeftPower(0);
                        robot.robotRightPower(0);
                        telemetry.addData("PID OUT:", df.format(0.00));
                        rotateComplete = true;
                    } else {
                        double output = PIDResult.getOutput();
                        robot.rightMotor_1.setPower(output);
                        robot.rightMotor_2.setPower(output);
                        robot.leftMotor_1.setPower(-output);
                        robot.leftMotor_2.setPower(-output);
                        telemetry.addData("PID OUT:", df.format(output) + " , " + df.format(-output));
                    }

                } else {
                    Log.w("navx", "YAW PID TIMEOUT");
                }
            }
            yawPIDController.enable(false);

            yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
            yawPIDController.enable(true);

           //drives straight until white line
           /* while (robot.color.alpha() < 5 && !Thread.currentThread().isInterrupted() && opModeIsActive()) {

                telemetry.addData("color", robot.color.alpha());
                telemetry.update();

                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.robotLeftPower(driveSpeed);
                        robot.robotRightPower(driveSpeed);
                        telemetry.addData("PID OUT:", df.format(0.00));
                    } else {
                        double output = PIDResult.getOutput();
                        robot.leftMotor_1.setPower(limit(driveSpeed + output));
                        robot.leftMotor_2.setPower(limit(driveSpeed + output));
                        robot.rightMotor_1.setPower(limit(driveSpeed - output));
                        robot.rightMotor_2.setPower(limit(driveSpeed - output));
                        telemetry.addData("MOTOR OUT:", df.format(limit(driveSpeed + output))
                                + " , " + df.format(limit(driveSpeed - output)));
                    }

                } else {
                    Log.w("navx", "YAW PID TIMEOUT");
                }
            }*/

           double colorOutput = 0;

           pidController.enable();
         //  robot.runWithEncoders();

           while(robot.rightMotor_2.getCurrentPosition() < 500 || robot.leftMotor_2.getCurrentPosition() < 500
                   && !Thread.currentThread().isInterrupted() && opModeIsActive())
           {
               colorOutput = pidController.getOutput();
               robot.leftMotor_1.setPower(limit(driveSpeed + colorOutput));
               robot.leftMotor_2.setPower(limit(driveSpeed + colorOutput));
               robot.rightMotor_1.setPower(limit(driveSpeed - colorOutput));
               robot.rightMotor_2.setPower(limit(driveSpeed - colorOutput));

               telemetry.addData("Color Output: ", pidController.getOutput());
               telemetry.update();
           }

           pidController.disable();

           //TODO: check to see what side of the line this program follows
           pidController.enable();
         //  robot.runWithoutEncoders();

     /*      while(robot.distance.getLightDetected() > 10 && !Thread.currentThread().isInterrupted())
           {
               colorOutput = pidController.getOutput();
               robot.leftMotor_1.setPower(limit(driveSpeed + colorOutput));
               robot.leftMotor_2.setPower(limit(driveSpeed + colorOutput));
               robot.rightMotor_1.setPower(limit(driveSpeed - colorOutput));
               robot.rightMotor_2.setPower(limit(driveSpeed - colorOutput));

               telemetry.addData("Color Output: ", pidController.getOutput());
               telemetry.update();
           }*/

           pidController.disable();


           //TODO: adjust color luminance value and servo values & check which color we are going for
         /*  if(robot.beaconColor.alpha() > 30)
               robot.pusherLeft.setPosition(1.0);
           else
               robot.pusherRight.setPosition(0.0);

*/
        if(isStopRequested()) {
            robot.robotLeftPower(0);
            robot.robotRightPower(0);
            telemetry.addData("Program Complete", "");
            telemetry.update();
          //  robot.color.enableLed(false);
        }

        }


    private double limit(double a) {
       return Math.min(Math.max(a, MIN_OUTPUT_VAL), MAX_OUTPUT_VAL);
    }
}
