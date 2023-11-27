package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


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
            tfDetection();
            int tagID = apDetection();

            telemetry.addData("tag id", tagID);
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

    public int apDetection () {
        List<AprilTagDetection> tagList = aprilT.getDetections();
        AprilTagDetection tag;

        if (tagList.size() > 1) {
            telemetry.addData("ERR: More than one tag detected:", tagList.size() + " tags");
            return 0;
        }

        // Step through the list of detections and display info for each one.
        tag = tagList.get(0);
        for (AprilTagDetection tag_ : tagList) {
            tag = tag_;
            if (tag.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return tag.id;
    }

    public void tfDetection() {
        List<Recognition> tfodList = tensor.getRecognitions();
        Recognition tfodObject;
        float x, y;

        telemetry.addData("# Objects Detected", JavaUtil.listLength(tfodList));
        for (Recognition item : tfodList) {
            tfodObject = item;
            telemetry.addLine("");
            // Display the label and confidence for the recognition
            telemetry.addData("Image", tfodObject.getLabel() + " (" + JavaUtil.formatNumber(tfodObject.getConfidence() * 100, 0) + " % Conf.)");
            // Display position of the object
            x = (tfodObject.getLeft() + tfodObject.getRight()) / 2;
            y = (tfodObject.getTop() + tfodObject.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(tfodObject.getWidth(), 0) + " x " + JavaUtil.formatNumber(tfodObject.getHeight(), 0));
        }
    }

}
