package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class RedF2Auto extends LinearOpMode {
    OpenCvCamera webcam;
    SignalPipeline signalPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(26)
                .build();

        Trajectory location1 = drive.trajectoryBuilder(forward.end())
                .strafeLeft(26)
                .build();

        Trajectory location3 = drive.trajectoryBuilder(forward.end())
                .strafeRight(26)
                .build();

        Trajectory location2 = drive.trajectoryBuilder(forward.end())
                .forward(25)
                .build();
        /*
        Trajectory location1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -34), Math.toRadians(90))
                .splineTo(new Vector2d(-12, -34), Math.toRadians(0))
                .build();

        Trajectory location2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -36), Math.toRadians(90))
                .build();

        Trajectory location3 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-33, -36), Math.toRadians(90))
                .splineTo(new Vector2d(-46, -36), Math.toRadians(0))
                .build();

         */

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

        /*
        int colour = signalPipeline.getColour();

        telemetry.addData("colour", colour);

         */

        waitForStart();

        if(isStopRequested()) return;

        /*
        while (opModeIsActive()) {
            int colour = signalPipeline.getColour();
            int colours[] = signalPipeline.getYCrCbVals();
            telemetry.addData("colour", colour);
            telemetry.addData("r", colours[0]);
            telemetry.addData("g", colours[1]);
            telemetry.addData("b", colours[2]);
            telemetry.update();
        }

         */

        int colour = signalPipeline.getColour();
        telemetry.addData("colour", colour);
        telemetry.update();

        drive.followTrajectory(forward);
        switch (colour) {
            case 1:
                telemetry.addLine("pos 1");
                drive.followTrajectory(location1);
                break;
            case 2:
                telemetry.addLine("pos 2");
                drive.followTrajectory(location2);
                break;
            case 3:
                telemetry.addLine("pos 3");
                drive.followTrajectory(location3);
                break;
        }
        telemetry.update();

        //drive.followTrajectory(location1);

        sleep(1000);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

}

class SignalPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    private Mat coneSubmat;
    //private Mat YCrCb;
    /*
    80, 100
    140, 140
    */
    private final Point point0 = new Point(140, 100);
    private final Point point1 = new Point(200, 140);
    private final Rect coneRect = new Rect(point0,point1);

    //int Y, Cr, Cb;
    int R, G, B;
    int[] colour = {0, 0, 0};

    @Override
    public void init(Mat firstFrame) {
        //YCrCb = new Mat();
        //Imgproc.cvtColor(firstFrame, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        coneSubmat = firstFrame.submat(coneRect);
    }
    @Override
    public Mat processFrame(Mat input) {

        //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2BGR);

        Imgproc.rectangle(input, coneRect, new Scalar(0,255,0), 3);

        //Y = (int) Core.mean(YCrCb).val[0];
        //Cb = (int) Core.mean(YCrCb).val[2];
        //Cr = (int) Core.mean(YCrCb).val[1];

        R = (int) Core.mean(coneSubmat).val[0];
        G = (int) Core.mean(coneSubmat).val[1];
        B = (int) Core.mean(coneSubmat).val[2];

        colour[0] = R;
        colour[1] = G;
        colour[2] = B;

        return input;
    }

    public int getColour() {

        int output = -1;

        int max = Math.max(R, B);
        int min = Math.min(R, B);
        if(max < 50) {
            //black
            output = 2;
        } else if(R-B > 30 || R > 180) {
            //red
            output = 1;
        } else if(B-R > 30 || B > 180) {
            //blue
            output = 3;
        }

        /*
        if(R > 160 && B > 160){
            output = 0;
        } else if(R > 170) {
            output = 1;
        } else if(B > 170) {
            output = 2;
        }

         */
        return output;
        /*
        int output = -1;
        if(Cb > 180 && Cr > 180){
            output = 0;
        } else if(Cr > 200) {
            output = 1;
        } else if(Cb > 200) {
            output = 2;
        }
        return output;

         */
    }

    public int[] getYCrCbVals() {
        return colour;
    }
}