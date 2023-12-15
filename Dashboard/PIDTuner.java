package org.firstinspires.ftc.teamcode.Dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "PIDTuner")
public class PIDTuner extends OpMode {

    // Declare OpMode members
    ElapsedTime runTime = new ElapsedTime();

    private BNO055IMU imu;
    DcMotor motor;
    Camera camera;
    boolean USE_WEBCAM;
    AprilTagProcessor aprilTag;
    VisionPortal vision;

    static final int TPR = 1440;

    int targetPosition = 0;

    public void init() {
        Gyroscope imu = hardwareMap.get(Gyroscope.class, "imu");

        motor = hardwareMap.get(DcMotor.class, "motor0");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Camera camera;
        boolean USE_WEBCAM;
        AprilTagProcessor aprilTag;
        VisionPortal vision;


        telemetry.addData("Status", "Initialized");
    }

    PIDController armControl = PIDController.create(0,0,0);

    @Override
    public void start() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PIDConstants obj = new PIDConstants();
        armControl.set(obj);

        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPower(0);

        if (gamepad1.y) {
            targetPosition = 2*TPR;
        }

        if (gamepad1.a) {
            targetPosition = 0;
        }


        motor.setPower(armControl.update(motor.getCurrentPosition(), targetPosition));

        telemetry.addData("motor tick", motor.getCurrentPosition());
        telemetry.addData("Baseline", 0);
        telemetry.addData("PID error", targetPosition - motor.getCurrentPosition());
        telemetry.addData("PID filter",  armControl.filter);
        telemetry.addData("PID Output x 1000", armControl.currentOut *1000);
        telemetry.addData("PID integral", armControl.integral);

        telemetry.update();
    }
}