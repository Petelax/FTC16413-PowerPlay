package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
0:fR
1:bR
2:fL
3:bL

bL: forward left odo
bR: strafe odo
fR: forward right odo

NEW:
fL:expansion hub 0
bL:expansion hub 1
 */

/*
control:
0: bL + left odo
1: fL + straft odo
2: elevator0 + arm encoder
3: intake

expansion:
0: arm
fR: right odo

left should be reversed
 */
@TeleOp
public class EncoderTest extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx gamepadEx;
    private RevIMU gyro;
    private OpenCvCamera webcam;

    @Override
    public void init() {
        fL = new Motor(hardwareMap, "fL", 448, 375);
        fR = new Motor(hardwareMap, "fR", 448, 375);
        bL = new Motor(hardwareMap, "bL", 448, 375);
        bR = new Motor(hardwareMap, "bR", 448, 375);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("NOT SHEESH", -1);

            }
        });

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, bL, bR);
        gyro = new RevIMU(hardwareMap);
        /*
        Timing.Timer timer = new Timing.Timer(1);
        timer.start();
        while(!timer.done()){
            telemetry.addData("timering", "lol");
            telemetry.update();
        }
        */
        telemetry.update();
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

        telemetry.addData("gyro", gyro.getHeading());

    }
}
