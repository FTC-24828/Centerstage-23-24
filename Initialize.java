package org.firstinspires.ftc.teamcode;


import android.util.Size;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Array;


public abstract class Initialize extends OpMode {

    private class genericList<T> {
        private T[] list;

        public genericList(Class<T> type, int capacity) {
            this.list = (T[]) Array.newInstance(type, capacity);
        }
    }

    public class InitObj {
        public <T> void initList (T[] list, Class<T> type, String name) {
            int id = 0;
            for (T item: list) {
                list[id] = hardwareMap.get(type, name + id);
                id++;
            }
            telemetry.addData(name + " status", "initialized");
        }

        public <T> void initItem (T item, Class<T> type, String name) {
            item = hardwareMap.get(type, name);
            telemetry.addData(name + " status", "initialized");
        }

        public void initCam(VisionPortal vis, String name, VisionProcessor... processor) {
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, name)).
                    setCameraResolution(new Size(640, 480)).
                    setStreamFormat(VisionPortal.StreamFormat.MJPEG).
                    enableLiveView(true).
                    setAutoStopLiveView(false);
            for (VisionProcessor p: processor) builder.addProcessors(p); //add processor to builder to portal
            vis = builder.build();
            telemetry.addData(name + " status", "initialized");
        }

        public void initCam(VisionPortal vis) throws Exception {
            throw new Exception("camera name not found");
        }
    }
}