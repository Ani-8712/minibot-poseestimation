package team3647.frc2024.util;

import java.util.ArrayList;
import java.util.function.Consumer;
import team3647.lib.team6328.VirtualSubsystem;

public class VisionController extends VirtualSubsystem {
    private final AprilTagCamera camera;
    private final Consumer<VisionData> addVisionData;

    private final ArrayList<VisionData> dataList = new ArrayList<VisionData>();

    public VisionController(Consumer<VisionData> addVisionData, AprilTagCamera camera) {
        this.addVisionData = addVisionData;
        this.camera = camera;
    }

    @Override
    public void periodic() {
        var inputs = camera.queueToInputs();

        inputs.ifPresent(
                (data) -> {
                    addVisionData.accept(data);
                    dataList.add(data);
                });
    }
}
