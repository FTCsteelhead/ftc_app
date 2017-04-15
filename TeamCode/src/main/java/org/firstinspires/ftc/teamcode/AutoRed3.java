package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Button Pusher - Red New", group = "Button")
//@Disabled
public class AutoRed3 extends LinearOpMode {

    private final int TOLERANCE_DEGREES = 2;

    private double MAX_OUTPUT_DRIVE = 1.0;
    private double MIN_OUTPUT_DRIVE = 0.5;
    private double MAX_OUTPUT_ROTATE = 0.5;
    private double MIN_OUTPUT_ROTATE = -0.5;
    private double MAX_OUTPUT_LINE = 0.25;
    private double MIN_OUTPUT_LINE = 0;

    private AutoRobotFunctions autoRobotFunctions;
    private Context appContext = null;

    private int whiteThreshold = 45;
    private int blueColor = 50;
    private int blackColor = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        appContext = hardwareMap.appContext;
        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

        robot.init(hardwareMap);

        whiteThreshold = robot.sharedPref.getInt(appContext.getString(R.string.White_Threshold), 45);
        blueColor = robot.sharedPref.getInt(appContext.getString(R.string.Blue_Color), 100);
        blackColor = robot.sharedPref.getInt(appContext.getString(R.string.Black_Threshold), 5);

        autoRobotFunctions = new AutoRobotFunctions(this, robot);
        autoRobotFunctions.setGyroDrivePID(0.018, 0.0001, 0.008);
     //   autoRobotFunctions.setGyroRotatePID(0.034, 0.0005, 0.0008);
        autoRobotFunctions.setGyroRotatePID(0.035, 0.00009, 0.000094);
        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


        telemetry.addData("STATUS:", "init complete–check state of gyro");
        telemetry.update();

        //wait for start of the match
        waitForStart();

        robot.robotForward();
        autoRobotFunctions.runWithEncoders(500, 1.0);

        autoRobotFunctions.MRRotate(20, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        //check to see if we miss the line
        if (autoRobotFunctions.MRDriveStraight(20, .75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 7200, 0.15,
                AutoRobotFunctions.StopConditions.COLOR, 25, -1)) {

            autoRobotFunctions.runWithEncoders(300, 1.0);

            autoRobotFunctions.MRRotate(75, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);


            autoRobotFunctions.PIDLineFollow(blackColor, whiteThreshold, 0.20, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                    AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.RIGHT);

            autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED, blueColor);

            robot.robotBackward();

            autoRobotFunctions.MRDriveStraight(90, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 2000, 0.15,
                    AutoRobotFunctions.StopConditions.ENCODER, 1000, -1);

            robot.robotForward();

            autoRobotFunctions.MRRotate(0, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

            robot.robotBackward();

           // autoRobotFunctions.runWithEncoders(500, 1.0);

            autoRobotFunctions.MRDriveStraight(0, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 600, 0.15,
                    AutoRobotFunctions.StopConditions.ENCODER, 500, -1);

            autoRobotFunctions.MRDriveStraight(0, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 2700, 0.10,
                    AutoRobotFunctions.StopConditions.COLOR, 20, -1);

            robot.robotForward();


           /* autoRobotFunctions.MRRotate(-90, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);*/

            autoRobotFunctions.runWithEncoders(200, 1.0);

            autoRobotFunctions.MRRotate(75, TOLERANCE_DEGREES,
                    MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

            autoRobotFunctions.PIDLineFollow(blackColor, whiteThreshold, 0.20, MIN_OUTPUT_LINE, MAX_OUTPUT_LINE, 0,
                    AutoRobotFunctions.StopConditions.BUTTON, AutoRobotFunctions.LineSide.RIGHT);

            autoRobotFunctions.pushButton(AutoRobotFunctions.Team.RED, blueColor);


            //shoot ball
            telemetry.addData("shooter power", robot.shooterMotorOn(true));
            telemetry.update();

            robot.sweeperMotor.setPower(-1.0);

            robot.robotBackward();

            autoRobotFunctions.MRDriveStraight(90, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 2495, 0.15,
                    AutoRobotFunctions.StopConditions.ENCODER, 2500, -1);

            robot.shooterServoDown(false);
            Thread.sleep(500);
            robot.shooterServoDown(true);
            Thread.sleep(800);
            robot.shooterServoDown(false);
            Thread.sleep(500);
            robot.shooterServoDown(true);
            robot.shooterMotorOn(false);
            robot.sweeperMotor.setPower(0.0);


            autoRobotFunctions.MRDriveStraight(20, 0.75,
                    MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 2495, 0.15,
                    AutoRobotFunctions.StopConditions.ENCODER, 2000, -1);


            //if the robot misses the line do this
        } else {
            telemetry.addData("You Missed the line!!", "<");
            telemetry.update();


        }

        autoRobotFunctions.close();

        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();

    }
}

