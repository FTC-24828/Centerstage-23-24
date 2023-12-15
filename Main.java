package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.util.List;

@TeleOp (name = "MainTeleOp")
public class Main extends OpMode {
    private IMU imu;
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();

    private ElapsedTime runtime = new ElapsedTime();
    static final int TPR = 1440;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu.resetYaw();
        runtime.reset();
    }

    @Override
    public void loop() {
        Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );

        //yaw, roll, pitch angles
        float Z = orientation.firstAngle;
        float Y = orientation.secondAngle;
        float X = orientation.thirdAngle;
        telemetry.addData("yaw", Math.toDegrees(Z));

        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

//        drivetrain.move(leftX, leftY);
        drivetrain.move(Drivetrain.localOrientation(leftX, leftY, Z));
        drivetrain.turn(rightX);

        if (gamepad1.a) {
        }

        if (gamepad1.y) {
        }

        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.update();
    }
}