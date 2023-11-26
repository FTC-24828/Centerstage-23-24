package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name = "MainAutonomous")
public class AutoMain extends LinearOpMode {

    static VisionPortal vision;
    AprilTagProcessor aprilT = new   AprilTagProcessor.Builder().
            setDrawAxes(true).          //draw 3D crosshair on tag
                    setDrawTagOutline(true).    //draw 3D cube projecting from tag
                    setDrawTagID(true).         //annotate tag detection with its ID
                    setDrawTagOutline(true).    //draw a 2D outline around tag
//                    setLensIntrinsics().        //Set the camera calibration parameters
//                    setNumThreads().            //Set the number of threads the tag detector should use
//                    setOutputUnits().           //Set the units you want translation and rotation data provided inside any AprilTagPoseRaw or AprilTagPoseFtc objects
//                    setTagFamily().             //Set the tag family this detector will be used to detect
//                    setTagFamily().             //Inform the detector about known tags.
        build();    //TODO: ADD setDecimation() VALUE
    TfodProcessor tensor = new TfodProcessor.Builder().
            build();


    @Override
    public void runOpMode() {
        InitObj initObj = new InitObj();
        initObj.initCam(vision, "camera0", aprilT, tensor);
        telemetry.addData("status", "waiting for start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("status", "runnning");
            telemetry.update();
        }
    }

    public class InitObj {
        public <T> void initList (T[] list, Class<T> type, String name) {
            int id = 0;
            for (T item: list) {
                list[id] = hardwareMap.get(type, name + id);
                id++;
            }
            telemetry.addData(name + " status", "initialized");
        }

        public <T> void initItem (T item, Class<T> type, String name) {
            item = hardwareMap.get(type, name);
            telemetry.addData(name + " status", "initialized");
        }

        public void initCam(VisionPortal vis, String name, VisionProcessor... processor) {
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, name)).
                    setCameraResolution(new Size(640, 480)).
                    setStreamFormat(VisionPortal.StreamFormat.MJPEG).
                    enableLiveView(true).
                    setAutoStopLiveView(false);
            for (VisionProcessor p: processor) builder.addProcessors(p);
            vis = builder.build();
            telemetry.addData(name + " status", "initialized");
        }

        public void initCam(VisionPortal vis) throws RuntimeException {
            throw new RuntimeException("camera name not found");
        }
    }
}


