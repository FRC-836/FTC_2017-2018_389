package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
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


    protected final long STEADY_STATE_SLEEP_TIME = 500;
    protected final double LIFT_POWER_DOWN = -0.10;
    protected final double LIFT_POWER_IDLE = 0.12;//Original arm: 0.09, double arm originally 0.18-too high
    protected final double LIFT_POWER_UP = 0.50;
    protected final double ENCODER_TURN_POWER = 0.19;//was 6
    protected final double ENCODER_DRIVE_POWER = 0.2;//was 0.5

    private final boolean USE_LEFT_ENCODER = true;
    protected final boolean RUN_TEST_CODE = false;

    private final double BEEP_EC_PER_FEET = 1304.8; // Encoder counts per Foot
    private final double BEEP_EC_PER_DEGREES_180 = 15.8;
    private final double BEEP_EC_PER_DEGREES_DEFAULT = BEEP_EC_PER_DEGREES_180;

    private final double I0_START = 0.0;
    private final double I1_START = 0.0;
    private final double I2_START = 0.0;
    private final double I3_START = 0.0;

    protected final double JEWEL_DRIVE_DISTANCE = 0.21; // feet
    protected final double SPECIAL_JEWEL_DRIVE_DISTANCE = 0.3;
    protected final double JEWEL_DRIVE_POWER = 0.10;

    private final double COLOR_UNCERTAINTY = 0.05; // Amount that (Red/Blue) > 1 or vice-versa to determine a color

    protected final long SLIGHT_LIFT_TIME = 150;
    protected final long SECOND_ROW_LIFT_TIME = 500;
    private  final long TIME_FOR_JEWEL = 500;
    private final long PAUSE_BETWEEN_TEST_CODE = 500;

    @Override
    public void initializeRobot() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
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
    protected void moveForwardEncoder(double distanceInFeet) {// move forward encoder based which allows you to drive using distance based.
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);//
        else
            moveForwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);//
    }

    protected void moveBackwardEncoder(double distanceInFeet) {// move backward  encoder based which allows you to drive using distance based.
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
        else
            moveBackwardRightEncoder(distanceInFeet, ENCODER_DRIVE_POWER);
    }

    protected void moveForwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveForwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveForwardRightEncoder(distanceInFeet, drivePower);
      }

    protected void moveBackwardEncoder(double distanceInFeet, double drivePower) {
        if (USE_LEFT_ENCODER)
            moveBackwardLeftEncoder(distanceInFeet, drivePower);
        else
            moveBackwardRightEncoder(distanceInFeet, drivePower);
    }

    private void moveForwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardRightEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backRightDrive.getCurrentPosition() - (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backRightDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backRightDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveForwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() + (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(drivePower, drivePower);
        while (backLeftDrive.getCurrentPosition() < targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void moveBackwardLeftEncoder(double distanceInFeet, double drivePower) {
        int targetPos = backLeftDrive.getCurrentPosition() - (int) (distanceInFeet * BEEP_EC_PER_FEET);
        setDrive(-drivePower, -drivePower);
        while (backLeftDrive.getCurrentPosition() > targetPos && opModeIsActive())
        {
            telemetry.addLine("Test: While Current Position < Goal");
            telemetry.addData("Current Position","%d",backLeftDrive.getCurrentPosition());
            telemetry.addData("Goal","%d",targetPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    protected void turnRight(double degreesOfTurn)//Allows us to to turn right encoder based which also allows us to turn distance based
    {
        if (USE_COMPASS_TURN) {
            compassTurn(degreesOfTurn);
        }
        else {
            if (USE_LEFT_ENCODER)
                turnRight_LeftEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
            else
                turnRight_RightEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
        }
    }

    protected void turnLeft(double degreesOfTurn)//Allows us to to turn left encoder based which also allows us to turn distance based
    {
        if (USE_COMPASS_TURN) {
            compassTurn(-degreesOfTurn);
        }
        else {
            if (USE_LEFT_ENCODER)
                turnLeft_LeftEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
            else
                turnLeft_RightEncoder(degreesOfTurn, BEEP_EC_PER_DEGREES_DEFAULT);
        }
    }

    protected void turnRight(double degreesOfTurn, double ecPerDegree)
    {
        if (USE_COMPASS_TURN) {
            compassTurn(degreesOfTurn);
        }
        else {
            if (USE_LEFT_ENCODER)
                turnRight_LeftEncoder(degreesOfTurn, ecPerDegree);
            else
                turnRight_RightEncoder(degreesOfTurn, ecPerDegree);
        }
    }

    protected void turnLeft(double degreesOfTurn, double ecPerDegree) {
        if (USE_COMPASS_TURN) {
            compassTurn(-degreesOfTurn);
        }
        else {
            if (USE_LEFT_ENCODER)
                turnLeft_LeftEncoder(degreesOfTurn, ecPerDegree);
            else
                turnLeft_RightEncoder(degreesOfTurn, ecPerDegree);
        }
    }

    private void turnRight_LeftEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backLeftDrive.getCurrentPosition();
        int targetPos = origPos + (int) (degreesOfTurn * ecPerDegree);
        setDrive(ENCODER_TURN_POWER, -ENCODER_TURN_POWER);
        while (backLeftDrive.getCurrentPosition() < targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnRight_RightEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backRightDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * ecPerDegree);
        setDrive(ENCODER_TURN_POWER, -ENCODER_TURN_POWER);
        while (backRightDrive.getCurrentPosition() > targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backRightDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backRightDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnLeft_LeftEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backLeftDrive.getCurrentPosition();
        int targetPos = origPos - (int) (degreesOfTurn * ecPerDegree);
        setDrive(-ENCODER_TURN_POWER, ENCODER_TURN_POWER);
        while (backLeftDrive.getCurrentPosition() > targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backLeftDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backLeftDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
        setDrive(0.0, 0.0);
    }

    private void turnLeft_RightEncoder(double degreesOfTurn, double ecPerDegree) {
        int origPos = backRightDrive.getCurrentPosition();
        int targetPos = origPos + (int) (degreesOfTurn * ecPerDegree);
        setDrive(-ENCODER_TURN_POWER, ENCODER_TURN_POWER);
        while (backRightDrive.getCurrentPosition() < targetPos && opModeIsActive()) {
            telemetry.addData("absolute data", "%d - %d - %d", origPos, backRightDrive.getCurrentPosition(), targetPos);
            telemetry.addData("relative data", "%d - %d - %d", 0, backRightDrive.getCurrentPosition() - origPos, targetPos - origPos);
            telemetry.update();
        }
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
        setIntake(I0_START, I1_START, I2_START, I3_START);

        waitForStart();
        runtime.reset();

        raiseJewelArm();
        closeBothIntakes();
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
    protected void scoreGlyph(boolean encoderUsed) {
        openBothIntakes();
        sleep(500);
        if(encoderUsed)
            moveBackwardEncoder(0.5, ENCODER_DRIVE_POWER);
        else
            moveStraightTime(-0.35, 500);
    }// TODO: Change if necessary, (estimates)
    protected void scoreOneMoreGlyph(){
        // Steps for Scoring the 2nd Glyph:
        // 1. Turn 180 Degrees
        turnLeft(180.0);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 2. Move forward 2 feet
        moveForwardEncoder(2.0);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 3. Pick up glyph
        closeBothIntakes();
        sleep(PAUSE_BETWEEN_TEST_CODE);
        timedLiftUp(SLIGHT_LIFT_TIME);
        sleep(100);
        // 4. Move backward 1 3/4 feet
        moveBackwardEncoder(1.5);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 5. Turn 180 Degrees
        turnRight(180.0);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 6. Lift a little bit so glyph doesn't drag
        timedLiftUp(SECOND_ROW_LIFT_TIME);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 7. Drive forward time based
        moveStraightTime(0.5, 1000);
        sleep(PAUSE_BETWEEN_TEST_CODE);
        // 8. Drop glyph
        // 9. Sleep
        // 10. Move backward
        scoreGlyph(true);
        // 11. Lower lift
        sleep(PAUSE_BETWEEN_TEST_CODE);
        timedLiftDown(SLIGHT_LIFT_TIME);
        // 12.(Optional)Drive forward to park

    }

    protected void timedLiftUp(long milliseconds){
        setLift(LIFT_POWER_UP);
        sleep(milliseconds);
        setLift(LIFT_POWER_IDLE);
    }
    protected void timedLiftDown(long milliseconds){
        setLift(LIFT_POWER_DOWN);
        sleep(milliseconds);
        setLift(0.0);
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
    }

    // compassTurn, setupIMU, and getCurrentDegrees from team 12888
    private float getCurrentDegrees()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    protected void compassTurn(double degrees) {
        float startPos = getCurrentDegrees();
        float goalAngle;
        if (degrees < 0.0)
        {
            degrees += 10.0;
            goalAngle = startPos - ((float) degrees);
            // Turning left
            setDrive(-COMPASS_TURN_POWER, COMPASS_TURN_POWER);
            if (goalAngle > 175.0) {
                goalAngle -= 360.0;
                sleep(COMPASS_PAUSE_TIME);
                while (getCurrentDegrees() >= startPos && opModeIsActive())
                {
                    telemetry.addData("Angle1","%.2f",getCurrentDegrees());
                    telemetry.update();
                }
            }
            while (getCurrentDegrees() < goalAngle && opModeIsActive())
            {
                telemetry.addData("Angle1","%.2f",getCurrentDegrees());
                telemetry.update();
            }
        }
        else
        {
            degrees -= 10.0;
            goalAngle = startPos - ((float) degrees);
            // Turning right
            setDrive(COMPASS_TURN_POWER, -COMPASS_TURN_POWER);
            if (goalAngle < -175.0) {
                goalAngle += 360.0;
                sleep(COMPASS_PAUSE_TIME);
                while (getCurrentDegrees() <= startPos && opModeIsActive())
                {
                    telemetry.addData("Angle1","%.2f",getCurrentDegrees());
                    telemetry.update();
                }
            }
            while (getCurrentDegrees() > goalAngle && opModeIsActive())
            {
                telemetry.addData("Angle1","%.2f",getCurrentDegrees());
                telemetry.update();
            }
        }
        setDrive(0.0, 0.0);
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
}
