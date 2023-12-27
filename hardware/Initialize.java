package org.firstinspires.ftc.teamcode.hardware;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


public class Initialize {
    public <T> void initList (HardwareMap hmap, T[] list, Class<T> type, String name) {
        int id = 0;
        for (T item : list) {
            list[id] = hmap.get(type, name + id); id++;
        }
    }

    //USELESS LOLOLOLOLLLLLLLLLLOLLLL
    public <T> void init(HardwareMap hmap, T[] item, Class<T> type, String name) {
        item[0] = hmap.get(type, name);
    }

    public void initCam(HardwareMap hmap, VisionPortal vis, String name, VisionProcessor... processor) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hmap.get(WebcamName.class, name)).
                setCameraResolution(new Size(640, 480)).
                setStreamFormat(VisionPortal.StreamFormat.MJPEG).
                enableLiveView(true).
                setAutoStopLiveView(false);
        for (VisionProcessor p: processor) builder.addProcessors(p); //add processor to builder to portal
        vis = builder.build();
    }
}