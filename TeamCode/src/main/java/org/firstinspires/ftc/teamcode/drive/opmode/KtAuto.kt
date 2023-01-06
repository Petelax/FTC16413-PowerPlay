package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
class KtAuto : LinearOpMode() {
    private lateinit var webcam: OpenCvCamera
    private lateinit var signalPipeline: SignalPipeline
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val drive = SampleMecanumDrive(hardwareMap)
        val startPose = Pose2d(-33.0, -60.0, Math.toRadians(90.0))
        drive.poseEstimate = startPose
        val forward = drive.trajectoryBuilder(startPose)
            .forward(26.0)
            .build()
        val location1 = drive.trajectoryBuilder(forward.end())
            .strafeLeft(26.0)
            .build()
        val location3 = drive.trajectoryBuilder(forward.end())
            .strafeRight(26.0)
            .build()
        val location2 = drive.trajectoryBuilder(forward.end())
            .forward(25.0)
            .build()
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
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )
        signalPipeline = SignalPipeline()
        webcam.setPipeline(signalPipeline)

        //webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(webcam, 30.0)
            }

            override fun onError(errorCode: Int) {
                telemetry.addLine("NOT SHEESH")
            }
        })

        /*
        int colour = signalPipeline.getColour();

        telemetry.addData("colour", colour);

         */
        waitForStart()
        if (isStopRequested) return

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
        val colour = signalPipeline.getColour()
        telemetry.addData("colour", colour)
        telemetry.update()
        drive.followTrajectory(forward)
        when (colour) {
            1 -> {
                telemetry.addLine("pos 1")
                drive.followTrajectory(location1)
            }
            2 -> {
                telemetry.addLine("pos 2")
                drive.followTrajectory(location2)
            }
            3 -> {
                telemetry.addLine("pos 3")
                drive.followTrajectory(location3)
            }
        }
        telemetry.update()

        //drive.followTrajectory(location1);
        sleep(1000)
        PoseStorage.currentPose = drive.poseEstimate
    }
}