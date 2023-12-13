package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class Main extends OpMode {
    private BNO055IMU imu;
    Drivetrain drivetrain = new Drivetrain();

    static Servo[] servo = new Servo[2];

    private ElapsedTime runtime = new ElapsedTime();
    static final int TPR = 1440;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
//        if (USE_CAM) initObj.initCam(vision, "camera0", aprilT);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        drivetrain.move(leftX, leftY);
        drivetrain.turn(rightX);

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

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.update();
    }




}