package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Localizer;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.util.WMath;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import javax.annotation.concurrent.GuardedBy;

public class WRobot {
    private static WRobot robot = null;

    //drivetrain
    public DcMotorEx[] motor = new DcMotorEx[4];
    public WEncoder[] motor_encoder = new WEncoder[4];
    public Localizer localizer;

    //arm
    public DcMotorEx lift;
    public WServo wrist;
    public WActuator arm_actuator;
    public WActuator wrist_actuator;
    public WEncoder arm_encoder;

    //intake
    public WServo claw_right;
    public WServo claw_left;

    //drone
    public WServo trigger;

    //hang
    public DcMotorEx hang_motor0;
    public DcMotorEx hang_motor1;
    public WEncoder hang_encoder;
    public WActuator hang_actuator;
    public WServo hook;

    private final Object imu_lock = new Object();
    @GuardedBy("imu_lock")
    private BNO055IMU imu;
    public Thread imu_thread;
    public double imu_offset = 0;
    private volatile double yaw = 0;

    public List<LynxModule> hubs;
    public LynxModule control_hub;
    public LynxModule expansion_hub;

    public VisionPortal vision_portal;
    public PropPipeline pipeline;

    private final ElapsedTime timer = new ElapsedTime();
    private double voltage = 12.0;

    private HardwareMap hardware_map;
    private Telemetry telemetry;

    //subsystems
    private List<WSubsystem> subsystems;
    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public Hang hang;
    public Drone drone;

    public  HashMap<Sensors.Encoder, Object> encoder_readings;
    public HashMap<Sensors.Sensor, Object> sensor_readings;

    //singleton declaration
    public static WRobot getInstance() {
        if (robot == null) robot = new WRobot();
        return robot;
    }


    //mapping and initializing hardware
    public void init(final HardwareMap hmap, Telemetry telemetry) {
        this.hardware_map = hmap;
        this.telemetry = (Global.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        if (Global.USING_IMU) {
            synchronized (imu_lock) {
                imu = hardware_map.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
            //imu_thread.setDaemon(true);
            resetYaw();
        }

        if (Global.USING_WEBCAM) {
            pipeline = new PropPipeline();
            vision_portal = new VisionPortal.Builder()
                    .setCamera(hardware_map.get(WebcamName.class, "Webcam"))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessors(pipeline)
                    .enableLiveView(Global.DEBUG)
                    .setAutoStopLiveView(true)
                    .build();
        }

        localizer = new Localizer(new Pose());

        encoder_readings = new HashMap<>();

        //drivetrain
        motor[0] = hardware_map.get(DcMotorEx.class, "motorFrontRight");
        motor[1] = hardware_map.get(DcMotorEx.class, "motorRearRight");
        motor[2] = hardware_map.get(DcMotorEx.class, "motorRearLeft");
        motor[3] = hardware_map.get(DcMotorEx.class, "motorFrontLeft");
        motor_encoder[0] = new WEncoder(new MotorEx(hardware_map, "motorFrontRight").encoder);
        motor_encoder[1] = new WEncoder(new MotorEx(hardware_map, "motorRearRight").encoder);
        motor_encoder[2] = new WEncoder(new MotorEx(hardware_map, "motorRearLeft").encoder);
        motor_encoder[3] = new WEncoder(new MotorEx(hardware_map, "motorFrontLeft").encoder);
        encoder_readings.put(Sensors.Encoder.LEFT_FRONT, 0);
        encoder_readings.put(Sensors.Encoder.RIGHT_FRONT, 0);
        encoder_readings.put(Sensors.Encoder.LEFT_REAR, 0);
        encoder_readings.put(Sensors.Encoder.RIGHT_REAR, 0);
        drivetrain.init(motor);
        localizer.init();


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

        //endgame subsystems
        if (!Global.IS_AUTO) {
            //drone
            if (drone != null) {
                trigger = new WServo(hardware_map.get(Servo.class, "trigger"));
                drone.init(trigger);
            }

            //hang
            if (hang != null) {
                hang_motor0 = hardware_map.get(DcMotorEx.class, "hang0");
                hang_motor1 = hardware_map.get(DcMotorEx.class, "hang1");
                hang_encoder = new WEncoder(new MotorEx(hardware_map, "hang0").encoder);
                encoder_readings.put(Sensors.Encoder.HANG_ENCODER, 0);
                hang_actuator = new WActuator(hang_motor0, hang_motor1);
                hook = new WServo(hardware_map.get(Servo.class, "hook"));
                hang.init(hang_motor0, hang_motor1, hook);
            }
        }
        //lynx hubs
        hubs = hardware_map.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber()))
                    control_hub = module;
            else expansion_hub = module;
        }

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

        for (WSubsystem subsystem : subsystems) { subsystem.periodic(); }
    }

    //read encoder values
    public void read () {
        if (arm != null) encoder_readings.put(Sensors.Encoder.ARM_ENCODER, arm_encoder.getPosition());
        if (hang != null) encoder_readings.put(Sensors.Encoder.HANG_ENCODER, hang_encoder.getPosition());

        if (Global.IS_AUTO) {
            encoder_readings.put(Sensors.Encoder.LEFT_FRONT, motor_encoder[0].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_FRONT, motor_encoder[1].getPosition());
            encoder_readings.put(Sensors.Encoder.LEFT_REAR, motor_encoder[2].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_REAR, motor_encoder[3].getPosition());
            localizer.update();
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
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

    public void startIMUThread(BooleanSupplier predicate) {
        if (Global.USING_IMU) {
            imu_thread = new Thread(() -> {
                while (predicate.getAsBoolean()) {
                    synchronized (imu_lock) {
                        yaw = WMath.wrapAngle(imu.getAngularOrientation().firstAngle - imu_offset);
                    }
                }
            });
            imu_thread.start();
        }
    }

    public void updateYaw() {
        yaw = WMath.wrapAngle(imu.getAngularOrientation().firstAngle - imu_offset);
    }

    public void resetYaw() {
        if (Global.USING_IMU) imu_offset = yaw;
    }

    public double getYaw() {
        return yaw;
    }

    public void clearBulkCache(@NonNull Global.Hub hub) {
        switch (hub) {
            case CONTROL_HUB:
                control_hub.clearBulkCache();
                break;

            case EXPANSION_HUB:
                expansion_hub.clearBulkCache();
                break;

            case BOTH:
                control_hub.clearBulkCache();
                expansion_hub.clearBulkCache();
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
