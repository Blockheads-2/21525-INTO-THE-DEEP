package org.firstinspires.ftc.teamcode.util;

import java.util.List;

public class Utility {
    /**
     * creates an implementation of telemetry data to be added to the dashboard
     * @param telemetryImplementationList a list for the implementation to be added to
     * @param implementation the data implementation
     */
    public static void registerTelemetryImplementation(List<Runnable> telemetryImplementationList, Runnable implementation) {
        telemetryImplementationList.add(implementation);
    }
}
