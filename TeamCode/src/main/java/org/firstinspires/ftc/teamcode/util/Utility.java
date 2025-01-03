package org.firstinspires.ftc.teamcode.util;


import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Kd;
import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Ki;
import static org.firstinspires.ftc.teamcode.util.Constants.Lift.Kp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Utility {
    private static double integralSum = 0;
    private static double lastError = 0;
    public static double control(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError);
        integralSum += error;

        lastError = error;

        return ((error * Kp) + (derivative * Kd) + (integralSum * Ki));
    }
}
