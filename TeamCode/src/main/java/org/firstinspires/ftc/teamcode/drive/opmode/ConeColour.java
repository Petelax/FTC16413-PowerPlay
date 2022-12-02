package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class ConeColour extends LinearOpMode {
    OpenCvCamera webcam;
    SignalPipeline signalPipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalPipeline = new SignalPipeline();
        webcam.setPipeline(signalPipeline);

        //webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("NOT SHEESH");

            }
        });

        waitForStart();

        while(opModeIsActive()) {
            int colour = signalPipeline.getColour();
            int colours[] = signalPipeline.getYCrCbVals();

            telemetry.addData("colour", signalPipeline.getColour());
            telemetry.addData("r", colours[0]);
            telemetry.addData("g", colours[1]);
            telemetry.addData("b", colours[2]);

            telemetry.update();
        }
    }
}
