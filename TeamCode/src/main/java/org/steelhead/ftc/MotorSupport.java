package org.steelhead.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Alec Matthews on 9/12/2016.
 * This function is a motor support library with methods to provide motor longevity.
 */

public class MotorSupport {

    /*TODO: add a motor ramp function using a p controller.
     *There is an example in the AutoRobotFunctions class */
    private DcMotor dcMotor;

    public MotorSupport(DcMotor motor) {
        dcMotor = motor;
    }

    public void motorRamp(double maxPower) {
        ElapsedTime delay = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        double power = 0;
        while(power < maxPower) {
            power = (0.002*delay.milliseconds());
            dcMotor.setPower(power);
        }
    }
}
