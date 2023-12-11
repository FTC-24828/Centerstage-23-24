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
    static DcMotor[] motor = new DcMotor[5];
    static Servo[] servo = new Servo[2];
    boolean USE_CAM = false;
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
    static final int TPR = 1440;

    @Override
    public void init() {
        //initialize hardware
        Initialize.InitObj initObj = new Initialize.InitObj();
        initObj.initList(motor, DcMotor.class, "motor");
        initObj.initList(servo, Servo.class, "servo");
        if (USE_CAM) initObj.initCam(vision, "camera0", aprilT);

        initObj = null;     //dereference initObj (delete from heap)

        //set drive train properties
        motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[2].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[3].setDirection(DcMotorSimple.Direction.REVERSE);

        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set motor arm
//        motor[4].setTargetPosition(0);
//        motor[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor[4].setDirection(DcMotor.Direction.FORWARD);
        motor[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //set servo properties
        servo[0].setDirection(Servo.Direction.REVERSE);
        servo[1].setDirection(Servo.Direction.FORWARD);

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
        double servo1Pos = servo[1].getPosition();



//        motor[4].setTargetPosition(TPR);
//        motor[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor[4].setPower(0.05);

        move(leftX, leftY);
        turn(rightX);
        motor[4].setPower(0);

        if (gamepad1.a) {
//            servo[0].setPosition(Range.clip(servo0Pos - 0.005, 0, 0.65));
//            servo[1].setPosition(Range.clip(servo1Pos - 0.005, 0, 0.65));
//            servo0Pos = servo[0].getPosition();
//            servo1Pos = servo[1].getPosition();

//            motor[4].setTargetPosition(0);
//            motor[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor[4].setPower(-0.2);
        }

        if (gamepad1.y) {
//            servo[0].setPosition(Range.clip(servo0Pos + 0.005, 0, 0.65));
//            servo[1].setPosition(Range.clip(servo1Pos + 0.005, 0, 0.65));
//            servo0Pos = servo[0].getPosition();
//            servo1Pos = servo[1].getPosition();

//            motor[4].setTargetPosition(TPR);
//            motor[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//             motor[4].setPower(0.5);
        }

        apDetection();
        tfDetection();

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.addData("Motor tick", motor[4].getCurrentPosition());
//        telemetry.addData("servo0 position", servo0Pos);
//        telemetry.addData("servo1 position", servo1Pos);
        telemetry.update();
    }

    public void move(double x, double y) {
        motor[0].setPower(Range.clip(y-x, -1.0, 1.0));
        motor[1].setPower(Range.clip(y+x, -1.0, 1.0));
        motor[2].setPower(Range.clip(y+x, -1.0, 1.0));
        motor[3].setPower(Range.clip(y-x, -1.0, 1.0));
    }

    public void turn(double x) {
        motor[0].setPower(-x);
        motor[1].setPower(x);
        motor[2].setPower(-x);
        motor[3].setPower(x);
    }

    public float PID(float state, float target) {
        float o = 0;
        return o;
    }

    public void apDetection () {
        List<AprilTagDetection> tagList = aprilT.getDetections();
        telemetry.addData("# AprilTags Detected", tagList.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection tag : tagList) {
            if (tag.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
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