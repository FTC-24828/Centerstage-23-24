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

@TeleOp (name = "Test")
public class Test extends OpMode {

    // Declare OpMode members
    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    Gyroscope imu;
    DcMotor motor;
    Camera camera;
    boolean USE_WEBCAM;
    AprilTagProcessor aprilTag;
    VisionPortal vision;

    static final int TPR = 1440;
    PIDController armControl = new PIDController(0.1, 0, 0);
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
        timer.reset();
    }

    public void loop() {
        //game stick xy
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX  =  gamepad1.right_stick_x;
        double rightY  =  gamepad1.right_stick_y;

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPower(0);

        if (gamepad1.y) {
            targetPosition = TPR;
        }

        if (gamepad1.a) {
            targetPosition = 0;
        }

        motor.setPower(armControl.update(motor.getCurrentPosition(), targetPosition));
        armControl.printAll();

        telemetry.addData("Run Time: ", runTime.toString());
        telemetry.addData("motor tick", motor.getCurrentPosition());
        telemetry.update();
        timer.reset();
    }

    public class PIDController {

        public double Kp, Ki, Kd, lastError, lastTarget, integral = 0;

        public double currentOut;       //for debugging output

        // gain values for proportional, integral, and derivative
        public PIDController (double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Kd = Kd;
            this.Ki = Ki;
        }

        // return and output based on the current state vs the target state
        public double update(double current, double target) {
            if (target != lastTarget) this.reset(target);
            double error = target - current;
            integral = integral + (error * timer.seconds());
            double output = Kp * error + Ki * integral + Kd * (error - lastError) / timer.seconds();
            lastError = error;
            currentOut = output;
            return output;
        }

        public void reset(double target) {
            integral = 0; lastError = 0; lastTarget = target;
            telemetry.addLine("changing target");
        }

        public void printAll() {
            telemetry.addData("Last error", lastError);
            telemetry.addData("Last target", lastTarget);
            telemetry.addData("integral", integral);
            telemetry.addData("current output", currentOut);
        }
    }
}