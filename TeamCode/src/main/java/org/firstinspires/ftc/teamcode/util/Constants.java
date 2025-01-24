package org.firstinspires.ftc.teamcode.util;

public class Constants {

    /** MILLIMETERS */
    public static double SPOOL_CIRCUMFERENCE = 112;
    public static double COUNTS_PER_REVOLUTION = 537.7;


    public static class Lift {
        public static double Kp = 0.005;
        public static double Ki = 0.0001;
        public static double Kd = 0.1;

        /** MILLIMETERS */
        public static double STRING_LENGTH = 231.0;
        public static double NECESSARY_REVOLUTIONS = 2.0625;
        public static double COMPLETE_REVOLUTION_COUNTS = 1109.00625;

        public static double TOP = -1350;
        public static double BOTTOM = 0;

        public static double UPPER_EXTENSION_BOUND = 0;
        public static double LOWER_EXTENSION_BOUND = 0.5;
    }
}
