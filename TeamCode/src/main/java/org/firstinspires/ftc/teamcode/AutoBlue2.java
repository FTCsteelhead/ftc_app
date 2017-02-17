package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.steelhead.ftc.Adafruit_GFX;
import org.steelhead.ftc.AutoRobotFunctions;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Robot hits both beacons, shoots 2 balls, pushes the yoga ball off the center, parks in the center
 */

@Autonomous(name = "Shoot - Blue", group = "Shoot")
//@Disabled
public class AutoBlue2 extends LinearOpMode {

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
        robot.setPoliceLED(true);
        waitForStart();

        Thread.sleep(10000);

        robot.robotBackward();

        robot.shooterPower(0.7);

      /*  autoRobotFunctions.MRDriveStraight(0, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 3500, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 500, -1);*/

        autoRobotFunctions.runWithEncoders(500, 1.0);

        robot.robotForward();

        autoRobotFunctions.MRRotate(-15, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

        autoRobotFunctions.runWithEncoders(2200, 1.0);


        Thread.sleep(500);
        robot.sweeperMotor.setPower(-1.0);

        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        Thread.sleep(1000);
        robot.shooterServoDown(false);
        Thread.sleep(500);
        robot.shooterServoDown(true);
        Thread.sleep(500);
        robot.shooterPower(0.0);

        robot.sweeperMotor.setPower(0.0);

        robot.robotForward();

        autoRobotFunctions.MRRotate(-65, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

       /* autoRobotFunctions.MRDriveStraight(50, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 3500, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 2000, -1);*/

        autoRobotFunctions.runWithEncoders(3000, 1.0);

        robot.robotForward();

        autoRobotFunctions.MRRotate(25, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

        autoRobotFunctions.runWithEncoders(200, 1.0);

        robot.robotForward();

        autoRobotFunctions.MRRotate(45, TOLERANCE_DEGREES,
                MIN_OUTPUT_ROTATE, MAX_OUTPUT_ROTATE);

        robot.robotBackward();

       /* autoRobotFunctions.MRDriveStraight(-45, 0.75,
                MIN_OUTPUT_DRIVE, MAX_OUTPUT_DRIVE, TOLERANCE_DEGREES, 0.0005, 3500, 0.15,
                AutoRobotFunctions.StopConditions.ENCODER, 2000, -1);*/

        autoRobotFunctions.runWithEncoders(1500, 1.0);



        autoRobotFunctions.close();

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                Adafruit_GFX gfx = new Adafruit_GFX(hardwareMap, "matrix", 8, 8);
                while (opModeIsActive()) {
                    gfx.animateBmp(R.drawable.firework, 19, 130, false);
                }
                gfx.close();
            }
        });
        thread.start();

        robot.setPoliceLED(false);
        robot.close();

        telemetry.addData("STATUS:", "Complete");
        telemetry.update();
    }
}

