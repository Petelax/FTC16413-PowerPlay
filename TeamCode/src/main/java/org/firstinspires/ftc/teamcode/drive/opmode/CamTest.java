package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@TeleOp
public class CamTest extends OpMode {
    OpenCvCamera webcam;
    GodPipeline godPipeline;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        godPipeline = new GodPipeline();
        webcam.setPipeline(godPipeline);

        //webcam.setMillisecondsPermissionTimeout(2500);
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

        telemetry.addData("SHEEEESH", 0);
        telemetry.update();

    }

    @Override
    public void loop() {
        int conePos = godPipeline.getColour();
        int[] colour = godPipeline.getYCrCbVals();
        telemetry.addData("B", colour[0]);
        telemetry.addData("G", colour[1]);
        telemetry.addData("R", colour[2]);
        telemetry.addData("conePos", conePos);
        telemetry.update();

    }
}

class GodPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    //private Mat coneSubmat;
    private Mat YCrCb;

    private final Point point0 = new Point(360/3f, 240/3f);
    private final Point point1 = new Point(360*(2f/3), 240*(2f/3));
    private final Rect coneRect = new Rect(point0,point1);

    int Y, Cr, Cb;
    int[] colour = {0, 0, 0};

    @Override
    public void init(Mat firstFrame) {
        YCrCb = new Mat();
        Imgproc.cvtColor(firstFrame, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        //coneSubmat = YCrCb.submat(coneRect);
    }
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2BGR);

        Imgproc.rectangle(input, coneRect, new Scalar(0,255,0), 4);

        Y = (int) Core.mean(YCrCb).val[0];
        Cb = (int) Core.mean(YCrCb).val[2];
        Cr = (int) Core.mean(YCrCb).val[1];

        colour[0] = Y;
        colour[1] = Cr;
        colour[2] = Cb;

        return YCrCb;
    }

    public int getColour() {
        int output = -1;
        if(Cb > 180 && Cr > 180){
            output = 0;
        } else if(Cr > 200) {
            output = 1;
        } else if(Cb > 200) {
            output = 2;
        }
        return output;
    }

    public int[] getYCrCbVals() {
        return colour;
    }
}
