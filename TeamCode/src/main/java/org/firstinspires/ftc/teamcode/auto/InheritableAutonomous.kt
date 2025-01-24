package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime


abstract class InheritableAutonomous : LinearOpMode() {
    override fun runOpMode() {}
}

class Lift(hardwareMap: HardwareMap) {
    private val lift: DcMotorEx

    init {
        lift = hardwareMap.get(DcMotorEx::class.java, "rightLift")
        lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lift.mode = DcMotor.RunMode.RUN_TO_POSITION
        lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lift.direction = DcMotorSimple.Direction.FORWARD
    }

    class LiftUp(position : Double) : Action {
        private var initialized = false
        override fun run(p: TelemetryPacket): Boolean {

            return false
        }
    }
    fun liftUp(position : Double): Action {
        return LiftUp(position)
    }


}

class Extension(hardwareMap: HardwareMap) {
    private val leftExtension: Servo
    private val rightExtension: Servo

    init {
        leftExtension = hardwareMap.get(Servo::class.java, "leftExtension")
        rightExtension = hardwareMap.get(Servo::class.java, "rightExtension")
        leftExtension.direction = Servo.Direction.REVERSE
        leftExtension.position = 0.0;
        rightExtension.position = 0.0;
    }

    fun setExtension(position: Double): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                leftExtension.position = position
                rightExtension.position = position

                return false
            }

        }
    }
}

class Pivot(hardwareMap: HardwareMap) {
    private val leftPivot: Servo
    private val rightPivot: Servo

    init {
        leftPivot = hardwareMap.get(Servo::class.java, "leftPivot")
        rightPivot = hardwareMap.get(Servo::class.java, "rightPivot")
        rightPivot.direction = Servo.Direction.REVERSE
        leftPivot.position = 0.0;
        rightPivot.position = 0.0;
    }
}

class Claw(hardwareMap: HardwareMap) {
    private val claw: Servo

    init {
        claw = hardwareMap.get(Servo::class.java, "claw")
        claw.position = 0.0;
    }

    fun open(): Action {
        return object : Action {
            private var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                claw.position = 0.2;
                return false
            }
        }
    }

    fun close(): Action {
        return object : Action {
            private var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                claw.position = 0.0;
                return false
            }
        }
    }
}

class Intake(hardwareMap : HardwareMap) {
    private val intake: CRServo

    init {
        intake = hardwareMap.get(CRServo::class.java, "intake")
    }

    fun roll(power: Double?, limit: Double): Action {
        return object : Action {
            private var initialized = false
            private val timer = ElapsedTime()

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    timer.reset()
                    initialized = true
                }

                val isFinished = timer.milliseconds() >= limit
                intake.power = power ?: 1.0
                if (isFinished) intake.power = 0.0;

                return !isFinished
            }
        }
    }
}