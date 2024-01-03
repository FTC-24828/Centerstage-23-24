package org.firstinspires.ftc.teamcode.opmode.tests.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.prop_pipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;


@Autonomous(name = "PipelineTest")
public class PipelineTest extends LinearOpMode {

    private PropPipeline prop_pipeline;
    private VisionPortal vision_portal;

    @Override
    public void runOpMode() throws InterruptedException {
        Global.COLOR = Global.Side.BLUE;
        Global.DEBUG = true;

        prop_pipeline = new PropPipeline();
        vision_portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(prop_pipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(prop_pipeline, 30);

        Scalar scale = new Scalar(1/1000000.0,1/1000000.0,1/1000000.0);
        while (opModeInInit()) {
            telemetry.addData("Location", prop_pipeline.getLocation());
            telemetry.addData("leftZone", prop_pipeline.left.mul(scale).toString());
            telemetry.addData("centerZone", prop_pipeline.center.mul(scale).toString());
            telemetry.addData("leftZone", prop_pipeline.leftColor);
            telemetry.addData("centerZone", prop_pipeline.centerColor);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", prop_pipeline.getLocation());
            telemetry.update();
        }
    }
}