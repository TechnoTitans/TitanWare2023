package frc.robot.utils.vision;

public enum CameraProperties {
    //TODO: calculate min target area
    SPINEL_UC10MPC_ND_OV9281(89.88, 1280, 720, 10),
    ARDUCAM_B0332_OV9281(80.36, 1280, 800, 10),
    MICROSOFT_LIFECAM_HD3000(68.5, 1280, 720, 10);

    private final double camDiagonalFOVDeg;
    private final int camResolutionWidthPx;
    private final int camResolutionHeightPx;
    private final double minTargetArea;

    CameraProperties(final double camDiagonalFOVDeg,
                     final int camResolutionWidthPx,
                     final int camResolutionHeightPx,
                     final double minTargetArea
    ) {
        this.camDiagonalFOVDeg = camDiagonalFOVDeg;
        this.camResolutionWidthPx = camResolutionWidthPx;
        this.camResolutionHeightPx = camResolutionHeightPx;
        this.minTargetArea = minTargetArea;
    }

    public double getCamDiagonalFOVDeg() {
        return camDiagonalFOVDeg;
    }

    public int getCamResolutionWidthPx() {
        return camResolutionWidthPx;
    }

    public int getCamResolutionHeightPx() {
        return camResolutionHeightPx;
    }

    public double getMinTargetArea() {
        return minTargetArea;
    }
}
