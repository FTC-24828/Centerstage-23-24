package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Controllers.Feedforward;
import org.firstinspires.ftc.teamcode.Controllers.MotionProfile;
import org.firstinspires.ftc.teamcode.Controllers.PIDF;
import org.firstinspires.ftc.teamcode.Other.GLOBAL;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Controller;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "MainTeleOp")
public class Main extends OpMode {
    private IMU imu;
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    Claw claw = new Claw();

    private ElapsedTime runtime = new ElapsedTime();
    int targetPosition = 5;
    public double INITIAL_YAW;         //TODO LINK BETWEEN THE TWO PROGRAMS

    //controllers
    private final PIDF armController = new PIDF(0.001, 0.02, 0.0001, 0.2, 1, 5);
    private final Feedforward armSupport = new Feedforward(0.07);

    public void init() {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        Controller controller = new Controller(gamepad1);

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

    public void start() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu.resetYaw();
        runtime.reset();
    }

    boolean ISPRESSED = false;
    boolean TRACKING = false;
    public void loop() {
        Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        );


        //yaw, roll, pitch angles
        float Z = orientation.firstAngle;
        telemetry.addData("yaw", Math.toDegrees(Z));

        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        drivetrain.move(Drivetrain.localOrientation(leftX * Math.abs(leftX), leftY * Math.abs(leftY), Z), rightX * Math.abs(rightX));
//        drivetrain.move(Drivetrain.localOrientation(leftX, leftY, Z), rightX);  //TODO ADD SHIFT MODE

        //180 is theta 0
        int currentPosition = arm.getPosition();
        double armAngle = (double)(currentPosition - 155) / GLOBAL.TPR * Math.PI;
        double wristAngle = (double)(Math.PI - armAngle) + 1.351;

        if (gamepad1.y) {
            targetPosition = 4*GLOBAL.TPR / 5;
            TRACKING = true;
        }

        //770
        if (gamepad1.a) {
            targetPosition = 5;
            arm.setWristAngle(0.6);
            TRACKING = false;
        }

        if (TRACKING) arm.setWristAngle(0.5 + wristAngle / Math.PI);

        if (gamepad1.b) {
            claw.setState(0, Claw.State.OPENED);
        }

        if (gamepad1.x) {
            claw.setState(1, Claw.State.OPENED);
        }

        if (gamepad1.right_bumper) { //TODO ADD A NEW GAMEPAD CLASS
            if (!ISPRESSED) {
                claw.switchState(0);
                claw.switchState(1);
            }
            ISPRESSED = true;
        } else {
            ISPRESSED = false;
        }


        arm.setPower(armController.update(currentPosition, targetPosition) * 1 +
                armSupport.update(Math.cos(armAngle)) * 1);


        telemetry.addData("arm position", currentPosition);
        telemetry.addData("arm angle", Math.toDegrees(armAngle));
        telemetry.addData("arm support", armSupport.getOutput());
        telemetry.addData("wrist angle", Math.toDegrees(wristAngle));


        telemetry.addData("wrist position", arm.getWristAngle());


        telemetry.addData("Status",  "Run Time: " + runtime.toString());
        telemetry.update();
    }
}