package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class WRobot {
    private static WRobot robot = null;

    //drivetrain
    public DcMotorEx[] motor = new DcMotorEx[4];
    public WEncoder[] motor_encoder = new WEncoder[4];

    //arm
    public DcMotorEx lift;
    public WServo wrist;
    public WActuator arm_actuator;
    public WActuator wrist_actuator;
    public WEncoder arm_encoder;

    //intake
    public WServo claw_right;
    public WServo claw_left;

    private IMU imu;
    public volatile double yaw;
    public List<LynxModule> hubs;

    public VisionPortal vision_portal;

    private final ElapsedTime timer = new ElapsedTime();
    private double voltage = 12.0;

    private HardwareMap hardware_map;
    private Telemetry telemetry;

    //subsystems
    private ArrayList<WSubsystem> subsystems;

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public Hang hang;
    public Drone drone;

    private HashMap<Sensors.Encoder, Object> encoder_readings;
    private HashMap<Sensors.Encoder, Object> sensor_readings;

    //singleton declaration
    public static WRobot getInstance() {
        if (robot == null) robot = new WRobot();
        return robot;
    }


    //mapping and initializing hardware
    public void init(final HardwareMap hmap, Telemetry telemetry, VisionProcessor... processor) {
        this.hardware_map = hmap;
        this.telemetry = (Global.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        if (Global.USING_IMU) {
            imu = hardware_map.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            ));
            imu.resetYaw();
        }

        if (Global.USING_WEBCAM) {
            vision_portal = new VisionPortal.Builder()
                    .setCamera(hardware_map.get(WebcamName.class, "Webcam"))
                    .setCameraResolution(new Size(1280, 720))
                    .setCamera(BuiltinCameraDirection.BACK)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessors(processor)
                    .enableLiveView(Global.DEBUG)
                    .setAutoStopLiveView(true)
                    .build();
        }

        this.encoder_readings = new HashMap<>();

        //drivetrain
        motor[0] = hardware_map.get(DcMotorEx.class, "motorFrontRight");
        motor[1] = hardware_map.get(DcMotorEx.class, "motorRearRight");
        motor[2] = hardware_map.get(DcMotorEx.class, "motorRearLeft");
        motor[3] = hardware_map.get(DcMotorEx.class, "motorFrontLeft");
        motor_encoder[0] = new WEncoder(new MotorEx(hardware_map, "motorFrontRight").encoder);
        motor_encoder[1] = new WEncoder(new MotorEx(hardware_map, "motorRearRight").encoder);
        motor_encoder[2] = new WEncoder(new MotorEx(hardware_map, "motorRearLeft").encoder);
        motor_encoder[3] = new WEncoder(new MotorEx(hardware_map, "motorFrontLeft").encoder);
        drivetrain.init(motor);


        //arm
        lift = hardware_map.get(DcMotorEx.class, "lift");

        arm_encoder = new WEncoder(new MotorEx(hardware_map, "lift").encoder);
        encoder_readings.put(Sensors.Encoder.ARM_ENCODER, 0);
        arm_actuator = new WActuator(() -> intSubscriber(Sensors.Encoder.ARM_ENCODER), lift)
                .setReadingOffset(-150);
        arm.init(lift);


        //intake
        wrist = new WServo(hardware_map.get(Servo.class, "wrist")).setWritingOffset(0.2);
        claw_right = new WServo(hardware_map.get(Servo.class, "servo0"));
        claw_left = new WServo(hardware_map.get(Servo.class, "servo1"));
        wrist_actuator = new WActuator(wrist::getPosition, wrist);
        intake.init(wrist, claw_left, claw_right);

        //lynx hubs
        hubs = hardware_map.getAll(LynxModule.class);
        hubs.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hubs.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
    }


    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems = new ArrayList<>();
        this.subsystems.addAll(Arrays.asList(subsystems));
        for (WSubsystem subsystem : subsystems) {
            switch (subsystem.getClass().getSimpleName()) {
                case "Drivetrain": drivetrain = (Drivetrain) subsystem; break;

                case "Arm": arm = (Arm) subsystem; break;

                case "Intake": intake = (Intake) subsystem; break;

                case "Hang": hang = (Hang) subsystem; break;

                case "Drone": drone = (Drone) subsystem; break;

                default:
                    throw new ClassCastException("Failed to add subsystem.");
            }
        }
    }


    public void periodic() {
        if (timer.seconds() > 5) {
            timer.reset();
            voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
//        if (Global.IS_AUTO) localizer.periodic();
    }

    //read encoder values
    public void read () {
        Thread imu_thread = new Thread(new IMU_GET_YAW_RUNNABLE(), "IMU-get-yaw-thread");
        imu_thread.start();

        encoder_readings.put(Sensors.Encoder.ARM_ENCODER, arm_encoder.getPosition());

        if (Global.IS_AUTO) {
            encoder_readings.put(Sensors.Encoder.LEFT_FRONT, motor_encoder[0].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_FRONT, motor_encoder[1].getPosition());
            encoder_readings.put(Sensors.Encoder.LEFT_REAR, motor_encoder[2].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_REAR, motor_encoder[3].getPosition());
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }

        if (Global.USING_IMU) {
            try {
                imu_thread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
        drivetrain = null;
        arm = null;
        intake = null;
        hang = null;
        drone = null;
    }

    public double getVoltage() {
        return voltage;
    }

    private class IMU_GET_YAW_RUNNABLE implements Runnable {
        @Override
        public void run() {
            try {
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }
    }


    private class IMU_RESET_YAW_RUNNABLE implements Runnable {
        @Override
        public void run() {
            try {
                imu.resetYaw();
            } catch (Exception e){
                Thread.currentThread().interrupt();
            }
        }
    }

    public void resetYaw() {
        new Thread(new IMU_RESET_YAW_RUNNABLE(), "IMU-reset-yaw-thread").start();
    }

    public void clearBulkCache(@NonNull Global.Hub hub) {
        switch (hub) {
            case CONTROL_HUB:
                hubs.get(0).clearBulkCache();
                break;

            case EXPANSION_HUB:
                hubs.get(1).clearBulkCache();
                break;

            case BOTH:
                hubs.get(0).clearBulkCache();
                hubs.get(1).clearBulkCache();
                break;
        }
    }


    public double doubleSubscriber(Sensors.Encoder topic) {
        Object value = encoder_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public double doubleSubscriber(Sensors.Sensor topic) {
        Object value = sensor_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.Encoder topic) {
        Object value = encoder_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.Sensor topic) {
        Object value = sensor_readings.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.Encoder topic) {
        return (boolean) encoder_readings.getOrDefault(topic, false);
    }

    public boolean boolSubscriber(Sensors.Sensor topic) {
        return (boolean) sensor_readings.getOrDefault(topic, false);
    }
}
