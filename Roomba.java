<<<<<<< HEAD
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "Roomba")
public abstract class Roomba extends OpMode {

    // Declare OpMode members
    Gyroscope imu;
    ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1, motor2, motor3, motor4;
    Camera camera;
    boolean USE_WEBCAM;
    AprilTagProcessor aprilTag;
    VisionPortal vision;

    @Override
    public void init() {
        Gyroscope imu = hardwareMap.get(Gyroscope.class, "imu");

        motor1 = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motor2 = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motor3 = hardwareMap.get(DcMotor.class, "motorRearRight");
        motor4 = hardwareMap.get(DcMotor.class, "motorRearLeft");

        Camera camera;
        boolean USE_WEBCAM;
        AprilTagProcessor aprilTag;
        VisionPortal vision;

        telemetry.addData("Status", "Initialized");
        runtime.reset();
    }

    public void move(double x, double y) {
        motor1.setPower(Range.clip(y+x, -1.0, 1.0));
        motor2.setPower(Range.clip(y-x, -1.0, 1.0));
        motor3.setPower(Range.clip(y-x, -1.0, 1.0));
        motor4.setPower(Range.clip(y+x, -1.0, 1.0));
    }

    public void turn(double x) {
        motor1.setPower(x);
        motor2.setPower(-x);
        motor3.setPower(x);
        motor4.setPower(-x);
    }

    @Override
    public void loop() {
        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);

        move(leftX, leftY);
        turn(rightX);

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.addData("left y", leftY);
        telemetry.addData("left x", leftX);
        telemetry.addData("right x", rightX);
        telemetry.update();
    }
=======
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "Roomba")
public abstract class Roomba extends OpMode {

    // Declare OpMode members
    Gyroscope imu;
    ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1, motor2, motor3, motor4;
    Camera camera;
    boolean USE_WEBCAM;
    AprilTagProcessor aprilTag;
    VisionPortal vision;

    @Override
    public void init() {
        Gyroscope imu = hardwareMap.get(Gyroscope.class, "imu");

        motor1 = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motor2 = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motor3 = hardwareMap.get(DcMotor.class, "motorRearRight");
        motor4 = hardwareMap.get(DcMotor.class, "motorRearLeft");

        Camera camera;
        boolean USE_WEBCAM;
        AprilTagProcessor aprilTag;
        VisionPortal vision;

        telemetry.addData("Status", "Initialized");
        runtime.reset();
    }

    public void move(double x, double y) {
        motor1.setPower(Range.clip(y+x, -1.0, 1.0));
        motor2.setPower(Range.clip(y-x, -1.0, 1.0));
        motor3.setPower(Range.clip(y-x, -1.0, 1.0));
        motor4.setPower(Range.clip(y+x, -1.0, 1.0));
    }

    public void turn(double x) {
        motor1.setPower(x);
        motor2.setPower(-x);
        motor3.setPower(x);
        motor4.setPower(-x);
    }

    @Override
    public void loop() {
        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);

        move(leftX, leftY);
        turn(rightX);

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.addData("left y", leftY);
        telemetry.addData("left x", leftX);
        telemetry.addData("right x", rightX);
        telemetry.update();
    }
>>>>>>> 47d4054 (Initial commit.)
}