package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public abstract class Tensor extends LinearOpMode {

  boolean USE_WEBCAM;
  TfodProcessor myTfodProcessor;
  VisionPortal myVisionPortal;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection.
    USE_WEBCAM = true;
    // Initialize TFOD before waitForStart.
    initTfod();
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetryTfod();
        // Push telemetry to the Driver Station.
        telemetry.update();
        if (gamepad1.dpad_down) {
          // Temporarily stop the streaming session.
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
   * Initialize TensorFlow Object Detection.
   */
  private void initTfod() {
    // First, create a TfodProcessor.
    myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
    // Next, create a VisionPortal.
    if (USE_WEBCAM) {
      myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "camera0"), myTfodProcessor);
    } else {
      myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myTfodProcessor);
    }
  }

  /**
   * Display info (using telemetry) for a detected object
   */
  private void telemetryTfod() {
    List<Recognition> myTfodRecognitions;
    Recognition myTfodRecognition;
    float x;
    float y;

    // Get a list of recognitions from TFOD.
    myTfodRecognitions = myTfodProcessor.getRecognitions();
    telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
    // Iterate through list and call a function to display info for each recognized object.
    for (Recognition myTfodRecognition_item : myTfodRecognitions) {
      myTfodRecognition = myTfodRecognition_item;
      // Display info about the recognition.
      telemetry.addLine("");
      // Display label and confidence.
      // Display the label and confidence for the recognition.
      telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
      // Display position.
      x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
      y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
      // Display the position of the center of the detection boundary for the recognition
      telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
      // Display size
      // Display the size of detection boundary for the recognition
      telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
    }
  }
}
