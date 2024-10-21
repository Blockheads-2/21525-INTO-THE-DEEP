package org.firstinspires.ftc.teamcode.util;


import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Kd;
import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Ki;
import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Kp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Utility {
    private static double integralSum = 0;
    private static double lastError = 0;
    protected static double control(double reference, double state, ElapsedTime time) {
        double error = reference - state;
        double derivative = (error - lastError) / time.seconds();
        integralSum += error * time.seconds();

        lastError = error;

        time.reset();

        return ((error * Kp) + (derivative * Kd) + (integralSum * Ki));
    }
}
