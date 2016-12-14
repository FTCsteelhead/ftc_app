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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the leftSpeed and rightSpeed Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Test", group="Steelhead")

public class SteelheadTeleOpTest extends OpMode{

    // use the defined hardware class for the robot
    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();


    double leftSpeed = 0;
    double rightSpeed = 0;
    double positionChange = 0.03;


    private boolean robotDirectionToggle = false;
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.robotForward();

        telemetry.addData("Status", "WAITING");    //
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {


        if(gamepad2.b)
            robot.sweeperMotor.setPower(0.0);

        if(gamepad2.a)
            robot.sweeperMotor.setPower(-1.0);



        if(gamepad2.x)
            robot.shooterPower(0.0);

        if(gamepad2.y)
            robot.shooterPower(0.7);


        if(gamepad2.dpad_up)
            robot.shooterServo.setPosition(0.0);

        if(gamepad2.dpad_down)
            robot.shooterServo.setPosition(1);


        leftSpeed = gamepad1.left_stick_y;
        rightSpeed = gamepad1.right_stick_y;

        robot.robotLeftPower(rightSpeed);
        robot.robotRightPower(leftSpeed);

        if(gamepad2.right_bumper)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() - positionChange );
        if(gamepad2.right_trigger > 0)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() + positionChange );

        if(gamepad2.left_bumper)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() + positionChange );
        if(gamepad2.left_trigger > 0)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() - positionChange );

        telemetry.addData("leftSpeed",  "%.2f", leftSpeed);
        telemetry.addData("rightSpeed", "%.2f", rightSpeed);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
