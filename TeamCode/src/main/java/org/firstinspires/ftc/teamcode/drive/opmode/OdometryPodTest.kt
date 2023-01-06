package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer

@TeleOp
class OdometryPodTest : OpMode() {
    private lateinit var leftOdo : Motor
    private lateinit var strafeOdo : Motor
    private lateinit var rightOdo : Motor

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        leftOdo = Motor(hardwareMap, "bL") /* left odo pod */
        strafeOdo = Motor(hardwareMap, "elevator0") /* strafe odo pod */
        rightOdo = Motor(hardwareMap, "odoPod0") /* right odo pod */

        //strafeOdo.encoder.setDirection(Motor.Direction.REVERSE)
        //leftOdo.encoder.setDirection(Motor.Direction.REVERSE)
        //rightOdo.encoder.setDirection(Motor.Direction.REVERSE)

        leftOdo.resetEncoder()
        strafeOdo.resetEncoder()
        rightOdo.resetEncoder()
    }

    override fun loop() {
        telemetry.addData("leftOdo", leftOdo.currentPosition)
        telemetry.addData("strafeOdo", strafeOdo.currentPosition)
        telemetry.addData("rightOdo", rightOdo.currentPosition)
        telemetry.addData("leftOdoInches", ticksToInches(leftOdo.currentPosition))
        telemetry.addData("strafeOdo", ticksToInches(strafeOdo.currentPosition))
        telemetry.addData("rightOdo", ticksToInches(rightOdo.currentPosition))
        telemetry.update()
    }

    private fun ticksToInches(ticks: Int):Double {
        return StandardTrackingWheelLocalizer.WHEEL_RADIUS * 2.0 * Math.PI * StandardTrackingWheelLocalizer.GEAR_RATIO * ticks.toDouble() / StandardTrackingWheelLocalizer.TICKS_PER_REV
    }
}