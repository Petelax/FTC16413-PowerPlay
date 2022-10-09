package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
0:fR
1:bR
2:fL
3:bL

bL: forward left odo
bR: strafe odo
fR: forward right odo

 */
@TeleOp
public class EncoderTest extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx gamepadEx;
    private RevIMU gyro;


    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", 448, 375);
        fR = new Motor(hardwareMap, "fR", 448, 375);
        bL = new Motor(hardwareMap, "bL", 448, 375);
        bR = new Motor(hardwareMap, "bR", 448, 375);

        drive = new MecanumDrive(fL, fR, bL, bR);
        gyro = new RevIMU(hardwareMap);
        gyro.init();
        //gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        /*
        drive.driveFieldCentric(
                gamepadEx.getLeftX(),
                gamepadEx.getLeftY(),
                gamepadEx.getRightX(),
                gyro.getHeading());

         */

        fR.set(gamepad1.dpad_up ? 0.5 : 0);
        fL.set(gamepad1.dpad_left ? 0.5 : 0);
        bR.set(gamepad1.dpad_right ? 0.5 : 0);
        bL.set(gamepad1.dpad_down ? 0.5 : 0);

        telemetry.addData("fL", fL.getCurrentPosition());
        telemetry.addData("fR", fR.getCurrentPosition());
        telemetry.addData("bL", bL.getCurrentPosition());
        telemetry.addData("bR", bR.getCurrentPosition());


    }
}
