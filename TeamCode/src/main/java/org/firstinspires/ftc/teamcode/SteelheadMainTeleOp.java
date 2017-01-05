/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the leftError and rightError Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Steelhead TeleOp", group = "Steelhead")

public class SteelheadMainTeleOp extends OpMode {

    // use the defined hardware class for the robot
    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

    private ElapsedTime rampTimer = new ElapsedTime();
    private ElapsedTime buttonTimer = new ElapsedTime();

    double leftError = 0;
    double rightError = 0;

    private final double buttonTime = 200; //milliseconds
    private final double rampTime = 50;//milliseconds
    private final double servoPositionChange = 0.03;
    private final double rampDistance = 20.0;
    private final double distanceMultiplier = 0.025;
    private final double rampMultiplier = 0.15;

    private boolean robotDirectionToggle = false;
    private boolean sweeperMotorToggle = false;
    private boolean shooterMotorToggle = false;

    private double workingLeftForwardSpeed = 0;
    private double workingRightForwardSpeed = 0;

    private final double MAX_SPEED = 0.5;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.robotForward();

        gamepad1.setJoystickDeadzone(0.08f);
        gamepad2.setJoystickDeadzone(0.08f);

        telemetry.addData("Status", "WAITING");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        buttonTimer.reset();
        rampTimer.reset();
    }

    @Override
    public void loop() {

        if (buttonTimer.milliseconds() >= buttonTime) {
            buttonTimer.reset();
            if (gamepad2.b && !sweeperMotorToggle) {
                robot.sweeperMotor.setPower(0.0);
                sweeperMotorToggle = true;
            } else if (gamepad2.b && sweeperMotorToggle) {
                robot.sweeperMotor.setPower(-1.0);
                sweeperMotorToggle = false;
            }

            if (gamepad2.x && !shooterMotorToggle) {
                robot.shooterPower(0.0);
                shooterMotorToggle = true;
            } else if (gamepad2.x && shooterMotorToggle) {
                robot.shooterPower(0.33);
                shooterMotorToggle = false;
            }

            if (gamepad2.dpad_up)
                robot.shooterServo.setPosition(0.4);

            if (gamepad2.dpad_down)
                robot.shooterServo.setPosition(0.55);

            if (gamepad1.left_bumper && !robotDirectionToggle) {
                robot.robotBackward();
                robotDirectionToggle = true;
            } else if (gamepad1.left_bumper && robotDirectionToggle) {
                robot.robotForward();
                robotDirectionToggle = false;
            }
        }

        if (gamepad1.atRest()) {
            workingLeftForwardSpeed = 0;
            workingRightForwardSpeed = 0;
        } else if (gamepad1.right_bumper) {
            //ENTER TURBO MODE
            workingLeftForwardSpeed = gamepad1.left_stick_y;
            workingRightForwardSpeed = gamepad1.right_stick_y;

        } else {
            workingLeftForwardSpeed = gamepad1.left_stick_y;
            workingRightForwardSpeed = gamepad1.right_stick_y;

            if (workingRightForwardSpeed > MAX_SPEED) {
                workingRightForwardSpeed = MAX_SPEED;
            } else if (workingRightForwardSpeed < -MAX_SPEED) {
                workingRightForwardSpeed = -MAX_SPEED;
            }

            if (workingLeftForwardSpeed > MAX_SPEED) {
                workingLeftForwardSpeed = MAX_SPEED;
            } else if (workingLeftForwardSpeed < -MAX_SPEED){
                workingLeftForwardSpeed = -MAX_SPEED;
            }
        }

        if (!robot.isRobotForward() && robot.range.getDistance(DistanceUnit.CM) < rampDistance) {
           /* if (workingLeftForwardSpeed <= -0.2) {
                workingLeftForwardSpeed += robot.range.getDistance(DistanceUnit.CM) * distanceMultiplier;
            } else */if (!gamepad1.atRest()){
                workingLeftForwardSpeed = -0.1;
           }
          /*  if (workingRightForwardSpeed <= -0.2) {
                workingRightForwardSpeed += robot.range.getDistance(DistanceUnit.CM) * distanceMultiplier;
            } else */if (!gamepad1.atRest()){
                workingRightForwardSpeed = -0.1;
            }
        }

        if (robot.isRobotForward()) {
            robot.robotLeftPower(workingRightForwardSpeed);
            robot.robotRightPower(workingLeftForwardSpeed);
        } else {
            robot.robotLeftPower(workingLeftForwardSpeed);
            robot.robotRightPower(workingRightForwardSpeed);
        }


        if (gamepad2.right_bumper)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() - servoPositionChange);
        if (gamepad2.right_trigger > 0)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() + servoPositionChange);

        if (gamepad2.left_bumper)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() + servoPositionChange);
        if (gamepad2.left_trigger > 0)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() - servoPositionChange);

        telemetry.addData("left joystick", gamepad1.left_stick_y);
        telemetry.addData("right joystick", gamepad1.right_stick_y);
        telemetry.addData("Distance", robot.range.getDistance(DistanceUnit.CM));
        telemetry.addData("leftError", "%.2f", leftError);
        telemetry.addData("rightError", "%.2f", rightError);
        telemetry.addData("working left speed", workingLeftForwardSpeed);
        telemetry.addData("working right speed", workingRightForwardSpeed);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.close();
    }

}
