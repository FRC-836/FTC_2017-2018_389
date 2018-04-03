package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class Autonomous_Parent extends Robot_Parent {

    //Vuforia Variables
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters vParameters;
    protected VuforiaTrackables relicTrackables;
    protected RelicRecoveryVuMark cryptoboxKey = null;
    private VuforiaTrackable relicTemplate = null;
    private VuforiaLocalizer vuforia;

    // IMU Variables
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private final boolean USE_COMPASS_TURN = true;

    private final double COMPASS_TURN_POWER = 0.19;
    // COMPASS_PAUSE_TIME - When using compassTurn, it waits COMPASS_PAUSE_TIME milliseconds before
    // using the compass to ensure the robot has begun moving.
    private final long COMPASS_PAUSE_TIME = 200;
    BNO055IMU imu;
    Orientation angles;

    enum ColorViewed {
        RED,
        BLUE,
        NEITHER
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor = null;

    protected final double TURN_STARTING_I_VALUE = 0.0;
    protected final double DRIVE_STARTING_I_VALUE = 0.0;
    protected final double JEWEL_DRIVE_DISTANCE = 0.21; // feet
    protected final double SPECIAL_JEWEL_DRIVE_DISTANCE = 0.3;
    protected final double JEWEL_DRIVE_POWER = 0.10;

    private final double COLOR_UNCERTAINTY = 0.05; // Amount that (Red/Blue) > 1 or vice-versa to determine a color

    private  final long TIME_FOR_JEWEL = 500;
    private final long PAUSE_BETWEEN_TEST_CODE = 500;

    // Telemtry Actions
    protected EncoderWatcher backLeftEncoderAction;
    protected EncoderWatcher backRightEncoderAction;

    @Override
    public void initializeRobot() {
        backLeftEncoderAction = new EncoderWatcher(telemetry, backLeftDrive, "Back Left Encoder");
        backRightEncoderAction = new EncoderWatcher(telemetry, backRightDrive, "Back Right Encoder");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        resetArmEncoder();
        resetDriveEncoders();
        setupVuMarkData();
        setupIMU();
        startUp();
    }

    @Override
    public void startRobot() {
        runAutonomous();
        telemetry.clear();
        telemetry.addData("Total runtime", "%6.3f seconds", runtime.seconds());//allows calculation of total runtime after the program ends to show on the phone.
        telemetry.update();
        while (opModeIsActive());
    }

    public void runAutonomous() {

    }

    protected void moveStraightTime(double setSpeed, long timeInMilliseconds) {
        setDrive(setSpeed, setSpeed);
        sleep(timeInMilliseconds);
        setDrive(0.0, 0.0);
    }

    private void raiseJewelArmMore() {// Allows us to raise our arm back closer to its starting position
        jewelArm.setPosition(JEWEL_ARM_FULLY_UP);
    }// Allows us to raise our arm back closer to its starting position

    protected ColorViewed getColorSeen() {
        if(colorSensor.blue() == 0){
            return ColorViewed.NEITHER;
        }
        else if(((double)colorSensor.red()) / ((double)colorSensor.blue()) > (1.0 + COLOR_UNCERTAINTY)) {
            return ColorViewed.RED;
        } else if (((double)colorSensor.red()) / ((double)colorSensor.blue()) < (1.0 - COLOR_UNCERTAINTY)) {
            return ColorViewed.BLUE;
        } else {
            return ColorViewed.NEITHER;
        }
    }

    protected void startUp() {
        raiseJewelArmMore(); // Locks jewel arm

        waitForStart();
        runtime.reset();

        raiseJewelArm();
    }

    protected RelicRecoveryVuMark getPictographKey() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime pictographTime = new ElapsedTime();
        pictographTime.reset();
        while ((vuMark == RelicRecoveryVuMark.UNKNOWN) && opModeIsActive() && (pictographTime.seconds() <= 5.0)) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        return vuMark;
    }
    protected void setupVuMarkData(){// Enables the VuMark data which allows to use the phone to scan the pictograph
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vParameters.vuforiaLicenseKey = "Ad/QI4f/////AAAAGTjpPxbdZUqSnVc3mldXKV0E3Ubo8UkPrp0l5P0ie1EXwbAiJNburExxvybAM/e5esxGLn3dl5zN73V9qcvBOROjy68/GQ8c0doo8ApEL127pzLSQEP6rZeq589EtDerLpgCqhsXSnU1hzLJ8S0UcgM9MeeUErzvfso6YAjLGZ9JXzLZHjXlX9lapHT64fBax9lZvMw5pmmZQE/j7oXqeamcdgnKyUn+wQN/3Gb+I2Ye7utY/LFJTiXFYesZsmE/eaq2mGnKVmqA4u6hvrSbEx/QudLqhl3nlXrAUPK+tx+5ersWNIB6OnNaRQApdYJb4mnO8qP8MgWhMIdT4Fuqd3WbitRP1NPUzqO+pJ63c36u";

        vParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vParameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }
    /*
    protected void scoreGlyph(boolean encoderUsed) {
        dropGlyph();
        sleep(500);
        if(encoderUsed)
            moveBackwardEncoder(0.5, ENCODER_DRIVE_POWER);
        else
            moveStraightTime(-0.35, 500);
    }
    protected void knockOffJewel(boolean isBlueTeam){
        switch (getColorSeen()) {
            case RED:
                // Moving Forward knocks off blue.
                telemetry.addLine("Saw red.");
                telemetry.update();

                if(isBlueTeam)
                    moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                else
                    moveForwardEncoder(SPECIAL_JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                sleep(TIME_FOR_JEWEL);
                //turnLeft(20.0);
                raiseJewelArm();
                sleep(1000);
                if(isBlueTeam) {
                    moveForwardEncoder(JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                } else {
                    moveBackwardEncoder(SPECIAL_JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                }
                sleep(TIME_FOR_JEWEL);
                //turnRight(30.0);
                telemetry.addLine("Saw red, done moving.");
                telemetry.update();
                break;
            case BLUE:
                // Turning Left knocks off red.
                telemetry.addLine("Saw blue.");
                telemetry.update();
                if(isBlueTeam) {
                    moveForwardEncoder(SPECIAL_JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                } else {
                    moveBackwardEncoder(JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                }
                sleep(TIME_FOR_JEWEL);
                //turnRight(20.0);
                raiseJewelArm();
                sleep(1000);
                if(isBlueTeam) {
                    moveBackwardEncoder(SPECIAL_JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                } else {
                    moveForwardEncoder(JEWEL_DRIVE_DISTANCE, JEWEL_DRIVE_POWER);
                }
                sleep(TIME_FOR_JEWEL);
                //turnLeft(20.0);
                sleep(1000);
                //turnLeft(20.0);
                telemetry.addLine("Saw blue, done moving.");
                telemetry.update();
                break;
            case NEITHER:
                // Just raise the arm
                telemetry.addLine("Unsure which color, lifting arm.");
                telemetry.update();
                raiseJewelArm();
                sleep(1000);
                //turnLeft(20.0);
                break;
        }
    }*/

    // compassTurn, setupIMU, and getCurrentDegrees from team 12888
    private float getCurrentDegrees()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    private void setupIMU()
    {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    private void drivePID(boolean isForward, double distance) {
        drivePID.resetPID(DRIVE_STARTING_I_VALUE);
        
        update();
    }

    private void turnPID(boolean isRight, double distance) {
        turnPID.resetPID(TURN_STARTING_I_VALUE);
    }

}
