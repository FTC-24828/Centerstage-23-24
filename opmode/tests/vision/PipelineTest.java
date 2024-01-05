package org.firstinspires.ftc.teamcode.opmode.tests.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;


@Autonomous(name = "PipelineTest")
public class PipelineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PropPipeline prop_pipeline;
        VisionPortal vision_portal;

        Global.SIDE = Global.Side.BLUE;
        Global.DEBUG = true;

        prop_pipeline = new PropPipeline();
        vision_portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(prop_pipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(prop_pipeline, 30);

        while (opModeInInit()) {
            telemetry.addData("Side", Global.SIDE);
            if (prop_pipeline.filter_range != null) {
                telemetry.addData("filter", prop_pipeline.filter_range[0] + "," +
                        prop_pipeline.filter_range[1] + "," +
                        prop_pipeline.filter_range[2] + "," +
                        prop_pipeline.filter_range[3] + "," +
                        prop_pipeline.filter_range[4] + "," +
                        prop_pipeline.filter_range[5]);
            }
            telemetry.addData("Location", prop_pipeline.getPropLocation());
            telemetry.addData("# of detections", prop_pipeline.getNumberOfDetection());
            telemetry.addData("leftZone", prop_pipeline.left_white);
            telemetry.addData("centerZone", prop_pipeline.center_white);
            telemetry.addData("threshold", prop_pipeline.threshold);
            telemetry.addData("FPS", vision_portal.getFps());
//            telemetry.addData("leftZone", prop_pipeline.leftColor);
//            telemetry.addData("centerZone", prop_pipeline.centerColor);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", prop_pipeline.getPropLocation());
            telemetry.update();
        }
    }
}