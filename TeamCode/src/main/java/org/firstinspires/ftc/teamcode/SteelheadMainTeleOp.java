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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Steelhead TeleOp", group="Steelhead")

public class SteelheadMainTeleOp extends OpMode{

    // use the defined hardware class for the robot
    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

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
        double left = 0;
        double right = 0;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        int num = 0;

        if(gamepad1.a) {
            num++;
        }

        if(gamepad1.b ) {
            num--;
        }


       // TODO: Check this it is giving me a null pointer exception
    /*  while(num % 2 == 1 && currentOpMode.opModeIsActive())
        {
            robot.robotBackward();
        }

        while(num % 2 == 0  && currentOpMode.opModeIsActive())
        {
            robot.robotForward();
        }


    /* while(gamepad1.left_stick_y != 0 && currentOpMode.opModeIsActive())
     {
         telemetry.addData("left power", left);
         telemetry.update();
         resetStartTime();
         while(getRuntime() < .25)
         {
             if(left < .5 && left > 0)
                 left = getRuntime() + .1;
             else if (left > -.5 && left < 0)
                 left = -(getRuntime() + .1);
             else
                 left = gamepad1.left_stick_y;
         }
     }

        while(gamepad1.right_stick_y != 0 && currentOpMode.opModeIsActive())
        {
            telemetry.addData("right power", right);
            telemetry.update();
            resetStartTime();
            while(getRuntime() < .25)
            {
                if(right < .5 && right > 0)
                 right = getRuntime() + .1;
                else if (right > -.5 && right < 0)
                    right = -(getRuntime() + .1);
                else
                    right = gamepad1.right_stick_y;
            }
        }
*/
        if(gamepad1.right_bumper) {
            left = (-gamepad1.left_stick_y);
            right = (-(gamepad1.right_stick_y));
        }
        else{
            left = (-gamepad1.left_stick_y)/2;
            right = (-(gamepad1.right_stick_y))/2;
        }

        robot.robotLeftPower(left);
        robot.robotRightPower(right);

        if(gamepad2.right_bumper)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() - .03 );
        if(gamepad2.right_trigger > 0)
            robot.pusherRight.setPosition(robot.pusherRight.getPosition() + .03 );

        if(gamepad2.left_bumper)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() + .03 );
        if(gamepad2.left_trigger > 0)
            robot.pusherLeft.setPosition(robot.pusherLeft.getPosition() - .03 );

        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
