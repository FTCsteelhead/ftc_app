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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.Adafruit_ColorSensor;
import org.steelhead.ftc.ColorPIDController;
import org.steelhead.ftc.HardwareSteelheadMainBot;

import java.text.DecimalFormat;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher", group = "Button")
//@Disabled
public class AutoButtonPusher extends LinearOpMode {

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navxDevice;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_UPDATE_RATE_HZ = 50;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double YAW_PID_P = 0.06;
    private final double YAW_PID_I = 0.0012;
    private final double YAW_PID_D = 0.85;

    private double MAX_OUTPUT_VAL = 0.15;
    private double MIN_OUTPUT_VAL = -0.15;

    private boolean calibrationComplete = false;
    private boolean rotateComplete = false;

    ColorSensor colorSensor;
    ColorPIDController pidController;

    TouchSensor touchSensor;

    Adafruit_ColorSensor beaconColor;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

    /* get the navx device from a device interface module named "dim"
     * Then create a new PID controller with the timestamped source YAW
     */

        navxDevice = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_UPDATE_RATE_HZ);



        //Configure the device

        robot.init(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get("color");
        pidController = new ColorPIDController(colorSensor, 3, 22);

        touchSensor = hardwareMap.touchSensor.get("touch");

        beaconColor = new Adafruit_ColorSensor(hardwareMap, "BColor");

        beaconColor.setLed(false);

        //Do this magic to make the color sensor work
        colorSensor.enableLed(true);
        colorSensor.enableLed(false);
        colorSensor.enableLed(true);
        telemetry.addData("Color sensor device id:", colorSensor.getManufacturer());
        telemetry.update();


        telemetry.addData("STATUS:", "init complete");
        telemetry.update();
        //wait for start of the match
        waitForStart();

        robot.leftMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!calibrationComplete) {
            calibrationComplete = !navxDevice.isCalibrating();
            telemetry.addData("CAL:", "Calibration in progress");
        }
        navxDevice.zeroYaw();

        robot.robotBackward();

        robot.leftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor_2.setTargetPosition(500);
        robot.rightMotor_2.setTargetPosition(500);

        robot.leftMotor_1.setPower(.25);
        robot.rightMotor_1.setPower(.25);
        robot.leftMotor_2.setPower(.25);
        robot.rightMotor_2.setPower(.25);


        while (opModeIsActive() && robot.leftMotor_2.isBusy() && robot.rightMotor_2.isBusy()) {
            telemetry.addData("DRIVING: ", "Moving %d encoder ticks", robot.leftMotor_2.getCurrentPosition());
            telemetry.update();
        }
        robot.leftMotor_1.setPower(0);
        robot.rightMotor_1.setPower(0);
        robot.leftMotor_2.setPower(0);
        robot.rightMotor_2.setPower(0);

        robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightMotor_1.setDirection(DcMotor.Direction.FORWARD);
        robot.rightMotor_2.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.robotSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            yawPIDController = new navXPIDController(navxDevice,
                    navXPIDController.navXTimestampedDataSource.YAW);

            yawPIDController.setContinuous(true);
            yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
            yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
            yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
            yawPIDController.setSetpoint(-45.0);
            yawPIDController.enable(true);

            navXPIDController.PIDResult PIDResult = new navXPIDController.PIDResult();
            int DEVICE_TIMEOUT_MS = 500;

            DecimalFormat df = new DecimalFormat("#.##");

            while ( opModeIsActive() && !rotateComplete && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(0);
                        robot.rightMotor_2.setPower(0);
                        robot.leftMotor_1.setPower(0);
                        robot.leftMotor_2.setPower(0);
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


            robot.robotBackward();

            robot.rightMotor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightMotor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftMotor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftMotor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
            yawPIDController.setSetpoint(135.0);

            double driveSpeed = 0.50;

            while (colorSensor.alpha() < 8 && opModeIsActive() && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(driveSpeed);
                        robot.rightMotor_2.setPower(driveSpeed);
                        robot.leftMotor_1.setPower(driveSpeed);
                        robot.leftMotor_2.setPower(driveSpeed);
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
                telemetry.addData("Color:", colorSensor.alpha());
                telemetry.update();
            }

            robot.robotRightPower(0.0);
            robot.robotLeftPower(0.0);

            robot.robotForward();

            robot.leftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftMotor_2.setTargetPosition(80);
            robot.rightMotor_2.setTargetPosition(80);

            robot.leftMotor_1.setPower(.25);
            robot.rightMotor_1.setPower(.25);
            robot.leftMotor_2.setPower(.25);
            robot.rightMotor_2.setPower(.25);


            while (opModeIsActive() && robot.leftMotor_2.isBusy() && robot.rightMotor_2.isBusy()) {
                telemetry.addData("ENC left : ", robot.leftMotor_2.getCurrentPosition());
                telemetry.addData("ENC right : ", robot.rightMotor_2.getCurrentPosition());
                telemetry.update();
            }
            robot.leftMotor_1.setPower(0);
            robot.rightMotor_1.setPower(0);
            robot.leftMotor_2.setPower(0);
            robot.rightMotor_2.setPower(0);

            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.robotBackward();
            /*yawPIDController.setSetpoint(-90.0);

            rotateComplete = false;

            while ( opModeIsActive() && !rotateComplete && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(0);
                        robot.rightMotor_2.setPower(0);
                        robot.leftMotor_1.setPower(0);
                        robot.leftMotor_2.setPower(0);
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
            }*/

            pidController.setPID(0.018, 0.05, 0.00203);
            pidController.setTolerance(0);

            double DRIVE_SPEED = 0.10;

            pidController.enable();
            sleep(500);
            while (opModeIsActive() && !touchSensor.isPressed()) {
                double output = pidController.getOutput();
                robot.leftMotor_1.setPower(limit(DRIVE_SPEED - output));
                robot.leftMotor_2.setPower(limit(DRIVE_SPEED - output));
                robot.rightMotor_1.setPower(limit(DRIVE_SPEED + output));
                robot.rightMotor_2.setPower(limit(DRIVE_SPEED + output));

                telemetry.addData("Output: ", pidController.getOutput());
                telemetry.update();
            }

            // Turn off and setup the pid controller for garbage collection
            pidController.disable();
            pidController = null;

            robot.robotLeftPower(0);
            robot.robotRightPower(0);

          //  robot.pusherLeft.setPosition(.5);

           runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() > 500) {
            telemetry.addData("beacon color", beaconColor.blueColor());
                telemetry.update();
            }


            if(beaconColor.blueColor() >  60)
                robot.pusherRight.setPosition(0.2);
            else
               robot.pusherLeft.setPosition(0.8);

            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < 1000);

            robot.pusherRight.setPosition(0.8);
            robot.pusherLeft.setPosition(0.2);

            robot.robotForward();

            robot.leftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            robot.leftMotor_2.setTargetPosition(700);
            robot.rightMotor_2.setTargetPosition(700);

            robot.leftMotor_2.setPower(.25);
            robot.rightMotor_2.setPower(.25);
            robot.leftMotor_1.setPower(.25);
            robot.rightMotor_1.setPower(.25);

            while (opModeIsActive() && robot.leftMotor_2.isBusy() && robot.rightMotor_2.isBusy()) {
                telemetry.addData("DRIVING: ", "Moving %d encoder ticks", robot.leftMotor_2.getCurrentPosition());
                telemetry.update();
            }
            robot.leftMotor_2.setPower(0);
            robot.rightMotor_2.setPower(0);
            robot.leftMotor_1.setPower(0);
            robot.rightMotor_1.setPower(0);

            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.robotBackward();

            //yawPIDController.setOutputRange(MIN_OUTPUT_VAL, MAX_OUTPUT_VAL);
            yawPIDController.setSetpoint(0.0);

            rotateComplete = false;

            while ( opModeIsActive() && !rotateComplete && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(0);
                        robot.rightMotor_2.setPower(0);
                        robot.leftMotor_1.setPower(0);
                        robot.leftMotor_2.setPower(0);
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

            robot.robotBackward();

            robot.leftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//moves forward do that it will move past the first line
           robot.leftMotor_2.setTargetPosition(-500);
            robot.rightMotor_2.setTargetPosition(-500);

            robot.leftMotor_2.setPower(.1);
            robot.rightMotor_2.setPower(.1);
            robot.leftMotor_1.setPower(.1);
            robot.rightMotor_1.setPower(.1);

            while (opModeIsActive() && robot.leftMotor_2.isBusy() && robot.rightMotor_2.isBusy()) {
                telemetry.addData("DRIVING: ", "Moving %d encoder ticks", robot.leftMotor_2.getCurrentPosition());
                telemetry.update();
            }

            robot.leftMotor_2.setPower(0);
            robot.rightMotor_2.setPower(0);
            robot.leftMotor_1.setPower(0);
            robot.rightMotor_1.setPower(0);


            robot.leftMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //drive until second line is found
            driveSpeed = .25;
            yawPIDController.setSetpoint(0.0);
            while (colorSensor.alpha() < 8 && opModeIsActive() && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(driveSpeed);
                        robot.rightMotor_2.setPower(driveSpeed);
                        robot.leftMotor_1.setPower(driveSpeed);
                        robot.leftMotor_2.setPower(driveSpeed);
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


            //turn 90 degrees to line
            yawPIDController.setSetpoint(-90.0);

            rotateComplete = false;

            while ( opModeIsActive() && !rotateComplete && !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(PIDResult, DEVICE_TIMEOUT_MS)) {
                    if (PIDResult.isOnTarget()) {
                        robot.rightMotor_1.setPower(0);
                        robot.rightMotor_2.setPower(0);
                        robot.leftMotor_1.setPower(0);
                        robot.leftMotor_2.setPower(0);
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

            //follow line until beacon
            pidController = new ColorPIDController(colorSensor, 3, 22);
            pidController.setPID(0.018, 0.05, 0.00203);
            pidController.setTolerance(0);

            pidController.enable();
            while (opModeIsActive() && !touchSensor.isPressed()) {
                double output = pidController.getOutput();
                robot.leftMotor_1.setPower(limit(DRIVE_SPEED - output));
                robot.leftMotor_2.setPower(limit(DRIVE_SPEED - output));
                robot.rightMotor_1.setPower(limit(DRIVE_SPEED + output));
                robot.rightMotor_2.setPower(limit(DRIVE_SPEED + output));

                telemetry.addData("Output: ", pidController.getOutput());
                telemetry.update();
            }

            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < 500) {
                telemetry.addData("beacon color", beaconColor.blueColor());
                telemetry.update();
            }


            if(beaconColor.blueColor() >  60)
                robot.pusherRight.setPosition(0.2);
            else
                robot.pusherLeft.setPosition(0.8);

            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < 1000);

            robot.pusherRight.setPosition(0.8);
            robot.pusherLeft.setPosition(0.2);
            robot.robotLeftPower(0);
            robot.robotRightPower(0);

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            navxDevice.close();
            colorSensor.enableLed(false);
            telemetry.addData("STATUS:", "Complete");
        }
    }

    private double limit(double a) {
       return Math.min(Math.max(a, MIN_OUTPUT_VAL), MAX_OUTPUT_VAL);
    }
}
