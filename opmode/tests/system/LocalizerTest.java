package org.firstinspires.ftc.teamcode.opmode.tests.system;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@Disabled
@TeleOp(name = "localizer test", group = "Utility")
public class LocalizerTest extends CommandOpMode {
    //initialize and getting the robot instance (singleton)
    private final WRobot robot = WRobot.getInstance();

    //declare controller variables but not initializing them (currently NULL)
    private GamepadEx controller;

    //called when the "init" button is pressed
    @Override
    public void initialize() {
        super.reset(); //reset the command scheduler (flushing out old commands from last opmode)

        Global.IS_AUTO = true;
        //additional global flags eg. USING_IMU, USING_DASHBOARD, DEBUG are placed here
        //if is auto, must declare color

        //initialize robot
        robot.addSubsystem(new Drivetrain());
        robot.init(hardwareMap, telemetry);

        //get controller
        controller = new GamepadEx(gamepad1);

        //display that initialization is complete
        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }

    //called when the play button is pressed
    @Override
    public void run() {
        robot.read(); //read values from encodes/sensors
        super.run(); //runs commands scheduled above

        //set the drivetrain's motor speed according to controller stick input
        robot.drivetrain.move(controller.getLeftX(), controller.getLeftY(), controller.getRightX());

        robot.periodic(); //calculations/writing data to actuators

        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Pose", robot.localizer.getPose().toString());
        telemetry.update();

        robot.write(); //write power to actuators (setting power to motors/servos)
        robot.clearBulkCache(Global.Hub.BOTH); //clear cache accordingly to get new read() values
    }

    //reset function, called when the opmode is stopped
    @Override
    public void reset() {
        super.reset(); //flush the command scheduler
        robot.reset();
        Global.resetGlobals();
    }
}

