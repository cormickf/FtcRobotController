
package org.firstinspires.ftc.teamcode.Background;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class VisionPathway {

    public enum ParkingPosition {
        UNDEFINED, ONE, TWO, THREE
    }

    protected final LinearOpMode linearOpMode;
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;

    protected ParkingPosition parkingPosition = ParkingPosition.UNDEFINED;

    public VisionPathway(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.hardwareMap = linearOpMode.hardwareMap;
        this.telemetry = linearOpMode.telemetry;
    }

    public ParkingPosition GetDetectedParkingPosition() { return this.parkingPosition; }

    public abstract void UpdateDetections();

}