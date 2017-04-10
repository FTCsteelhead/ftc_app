package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Robot hits both beacons, shoots 2 balls, pushes the yoga ball off the center, parks in the center
 */

@Autonomous(name = "Shoot - Red", group = "Shoot")
@Disabled
public class AutoRed2 extends LinearOpMode {

    private final int TOLERANCE_DEGREES = 2;

    private double MAX_OUTPUT_DRIVE = 1.0;
    private double MIN_OUTPUT_DRIVE = 0.5;
    private double MAX_OUTPUT_ROTATE = 0.25;
    private double MIN_OUTPUT_ROTATE = -0.25;
    private double MAX_OUTPUT_LINE = 0.25;
    private double MIN_OUTPUT_LINE = -0.25;



    private AutoRobotFunctions autoRobotFunctions;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();

        robot.init(hardwareMap);
        autoRobotFunctions = new AutoRobotFunctions(this, robot);

        autoRobotFunctions.setGyroDrivePID(0.018, 0.0001, 0.008);
        autoRobotFunctions.setGyroRotatePID(0.0327, 0.0005, 0.0008);

        autoRobotFunctions.setColorPID(0.018, 0.05, 0.00203);


        telemetry.addData("STATUS:", "init complete–check state of gyro");
        telemetry.update();

        robot.shooterServo.setPosition(1.0);

        //wait for start of the match
        waitForStart();

        Thread.sleep(10000);

        robot.robotBackward();

        robot.shooterMotorOn(true);



      //  autoRobotFunctions.runWithEncoders(500, 1.0);

        autoRobotFunctions.MRDriveStraight(0, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 500, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 500, -1);

        robot.robotForward();

        autoRobotFunctions.MRRotate(15, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

    //    autoRobotFunctions.runWithEncoders(2200, 1.0);

        autoRobotFunctions.MRDriveStraight(15, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 2100, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 2100, -1);

        Thread.sleep(500);
        robot.sweeperMotor.setPower(-1.0);

        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        robot.shooterMotorOn(false);
        Thread.sleep(5000);
        robot.shooterMotorOn(true);
        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        Thread.sleep(500);
        robot.shooterMotorOn(false);

        robot.sweeperMotor.setPower(0.0);

        robot.robotForward();

        autoRobotFunctions.MRRotate(20, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

        autoRobotFunctions.MRDriveStraight(20, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 1100, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 1100, -1);



        autoRobotFunctions.close();

        robot.close();


       telemetry.addData("STATUS:", "Complete");
       telemetry.update();
    }
}

