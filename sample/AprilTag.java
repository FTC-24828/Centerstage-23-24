package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AprilTag extends LinearOpMode {

  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  VisionPortal myVisionPortal;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation.
    USE_WEBCAM = true;
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetryAprilTag();
        // Push telemetry to the Driver Station.
        telemetry.update();
        if (gamepad1.dpad_down) {
          // Temporarily stop the streaming session. This can save CPU
          // resources, with the ability to resume quickly when needed.
          myVisionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
          // Resume the streaming session if previously stopped.
          myVisionPortal.resumeStreaming();
        }
        // Share the CPU.
        sleep(20);
      }
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    // First, create an AprilTagProcessor.
    myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    // Next, create a VisionPortal.
    if (USE_WEBCAM) {
      myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
    } else {
      myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myAprilTagProcessor);
    }
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }
}
