package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp (name = "MainTeleOp")
public class Main extends Initialize implements RunTime.RunTimeMethods {
    Gyroscope imu;
    static DcMotor[] motor = new DcMotor[4];
    static Servo[] servo = new Servo[1];
    boolean USE_CAM = true;
    static VisionPortal vision;
    AprilTagProcessor aprilT = new AprilTagProcessor.Builder().
                    setDrawAxes(true).          //draw 3D crosshair on tag
                    setDrawTagOutline(true).    //draw 3D cube projecting from tag
                    setDrawTagID(true).         //annotate tag detection with its ID
                    setDrawTagOutline(true).    //draw a 2D outline around tag
//                    setLensIntrinsics().        //Set the camera calibration parameters
//                    setNumThreads().            //Set the number of threads the tag detector should use
//                    setOutputUnits().           //Set the units you want translation and rotation data provided inside any AprilTagPoseRaw or AprilTagPoseFtc objects
//                    setTagFamily().             //Set the tag family this detector will be used to detect
//                    setTagFamily().             //Inform the detector about known tags.
                    build();

    //TODO: ADD setDecimation() VALUE

    TfodProcessor tensor = new TfodProcessor.Builder().
                    build();

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        //initialize hardware
        Initialize.InitObj initObj = new Initialize.InitObj();
        initObj.initList(motor, DcMotor.class, "motor");
        initObj.initList(servo, Servo.class, "servo");
        if (USE_CAM) initObj.initCam(vision, "camera0", aprilT);

        initObj = null;     //dereference initObj (delete from heap)

        //set motor direction for X-drive
        motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[3].setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        runtime.reset();
    }

    public void loop() {
        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        double servo0Pos = servo[0].getPosition();

        telemetry.addData("Status", "Running");

        move(leftX, leftY);
        turn(rightX);

        if (gamepad1.a) {
            servo[0].setPosition(servo0Pos + 0.01);
            servo0Pos = servo[0].getPosition();
        }

        if (gamepad1.y) {
            servo[0].setPosition(servo0Pos - 0.01);
            servo0Pos = servo[0].getPosition();
        }

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.addData("servo position", servo0Pos);
        telemetry.update();
    }

    public void move(double x, double y) {
        motor[0].setPower(Range.clip(y+x, -1.0, 1.0));
        motor[1].setPower(Range.clip(y-x, -1.0, 1.0));
        motor[2].setPower(Range.clip(y-x, -1.0, 1.0));
        motor[3].setPower(Range.clip(y+x, -1.0, 1.0));
    }

    public void turn(double x) {
        motor[0].setPower(x);
        motor[1].setPower(-x);
        motor[2].setPower(x);
        motor[3].setPower(-x);
    }

    public void apDetection () {
        List<AprilTagDetection> currentDetections = aprilT.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public void tfDetection() {
        List<Recognition> tfodList;
        Recognition tfodObject;
        float x, y;

        // Get a list of recognitions from TFOD
        tfodList = tensor.getRecognitions();
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