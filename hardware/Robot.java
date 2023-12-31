package org.firstinspires.ftc.teamcode.hardware;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WActuator;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class Robot {
    private static Robot robot = null;

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
    public double yaw;
    public List<LynxModule> hubs;

    private ElapsedTime timer = new ElapsedTime();
    private double voltage = 12.0;

    private HardwareMap hardware_map;
    private Telemetry telemetry;

    //subsystems
    private ArrayList<WSubsystem> subsystems = new ArrayList<>();

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public Hang hang;
    public Drone drone;

    private HashMap<Sensors.Encoder, Object> encoder_readings;
    private HashMap<Sensors.Encoder, Object> sensor_readings;

    //singleton declaration
    public static Robot getInstance() {
        if (robot == null) robot = new Robot();
        return robot;
    }


    //mapping and initializing hardware
    public void init(final HardwareMap hmap, Telemetry telemetry) {
        this.hardware_map = hmap;
        this.telemetry = (Global.USING_DASHBOARD) ? new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry()) : telemetry;

        imu = hardware_map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        this.encoder_readings = new HashMap<>();

        Initialize initObj = new Initialize();
        //drivetrain
//        initObj.initList(hardware_map, motor, DcMotorEx.class, "motor");
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
        wrist = new WServo(hardware_map.get(Servo.class, "wrist")).setWritingOffset(0.3);
        claw_right = new WServo(hardware_map.get(Servo.class, "servo0"));
        claw_left = new WServo(hardware_map.get(Servo.class, "servo1"));
        wrist_actuator = new WActuator(wrist::getPosition, wrist);
        intake.init(wrist, claw_left, claw_right);

        //subsystems
//        drivetrain = new Drivetrain();
//        arm = new Arm();
//        intake = new Intake();
//        addSubsystem(drivetrain, arm, intake);      //TODO MOVE THIS FUNC OUTSIDE INTO OPMODES FOR TESTING

        hubs = hardware_map.getAll(LynxModule.class);
        hubs.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hubs.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        clearBulkCache(Global.Hub.BOTH);

        voltage = hardware_map.voltageSensor.iterator().next().getVoltage();
    }

    public void addSubsystem(WSubsystem... subsystems) {
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
        encoder_readings.put(Sensors.Encoder.ARM_ENCODER, arm_encoder.getPosition());
        if (Global.IS_AUTO) {
            encoder_readings.put(Sensors.Encoder.LEFT_FRONT, motor_encoder[0].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_FRONT, motor_encoder[1].getPosition());
            encoder_readings.put(Sensors.Encoder.LEFT_REAR, motor_encoder[2].getPosition());
            encoder_readings.put(Sensors.Encoder.RIGHT_REAR, motor_encoder[3].getPosition());
        }

//        if (Global.USING_IMU) getYaw();
        Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        ); yaw = orientation.firstAngle;

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
//        robot = new Robot();
    }

    public double getVoltage() {
        return voltage;
    }

    private double getYaw() {
        Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS
        ); return orientation.firstAngle;
    }

    public void resetYaw() {
        imu.resetYaw();
    }


    public List<LynxModule> getModules() {
        return hubs;
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
        Object value = encoder_readings.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.Sensor topic) {
        Object value = sensor_readings.getOrDefault(topic, 0);
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
