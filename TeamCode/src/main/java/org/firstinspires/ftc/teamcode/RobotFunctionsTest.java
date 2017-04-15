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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Concept: Test Functions", group = "Concept")
//@Disabled
public class RobotFunctionsTest extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();

  private double MAX_OUTPUT_DRIVE = 1.0;
  private double MIN_OUTPUT_DRIVE = 0.5;
  private double MAX_OUTPUT_LINE = 0.25;
  private double MIN_OUTPUT_LINE = 0;
  private final int TOLERANCE_DEGREES = 2;


  private double MAX_OUTPUT_ROTATE = 0.25;
  private double MIN_OUTPUT_ROTATE = -0.25;

  private AutoRobotFunctions autoRobotFunctions;
  private Context appContext = null;

  private int whiteThreshold = 45;
  private int blueColor = 100;
  private int blackColor = 5;

  @Override
  public void runOpMode() throws InterruptedException {
    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

    robot.init(hardwareMap);
    AutoRobotFunctions autoRobotFunctions = new AutoRobotFunctions(this, robot);

      autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


    robot.robotForward();

    telemetry.addData("STATUS:", "init complete–check state of gyro");
    telemetry.update();

    waitForStart();

      autoRobotFunctions.PIDLineFollow(blackColor, whiteThreshold, 0.25, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
              AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.RIGHT);



      robot.close();

  }
}
