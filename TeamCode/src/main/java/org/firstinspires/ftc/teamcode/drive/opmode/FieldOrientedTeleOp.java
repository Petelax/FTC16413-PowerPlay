package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FieldOrientedTeleOp extends OpMode {

    private Motor fL, fR, bL, bR, elevator0, elevator1;
    private MecanumDrive drive;
    private GamepadEx gamepadEx;
    private RevIMU gyro;
    

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", 448, 375);
        fR = new Motor(hardwareMap, "fR", 448, 375);
        bL = new Motor(hardwareMap, "bL", 448, 375);
        bR = new Motor(hardwareMap, "bR", 448, 375);
        elevator0 = new Motor(hardwareMap, "elevator0", Motor.GoBILDA.RPM_435);
        elevator1 = new Motor(hardwareMap, "elevator1", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, bL, bR);
        gyro = new RevIMU(hardwareMap);
        gyro.init();
        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveFieldCentric(
                gamepadEx.getLeftX(),
                gamepadEx.getLeftY(),
                gamepadEx.getRightX(),
                gyro.getHeading());

        double elevator = gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        elevator0.set(elevator);
        elevator1.set(elevator);

        telemetry.addData("gyro", gyro.getHeading());
        telemetry.update();
    }
}
