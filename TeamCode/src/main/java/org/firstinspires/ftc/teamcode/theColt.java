package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import java.io.File;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
/**
 * Created by Drew from 6547 on 10/17/2018.
 */

@Disabled
class theColt extends LinearOpMode{
    final double encoderTicksPerRotation=510;
    final double circumferenceOfWheel=12.566370614359172;

    final double linearSlideMaxEncoder=-3850;
    final double armMaxEncoder=3850;
    double angleZzeroValue=0;

    MiniPID armPID = new MiniPID(.30, 0.00, 0);

    MiniPID slidePID = new MiniPID(.30, 0.00, 0);

    static double lastPosX=40;
    static double lastPosY=40;

    static final String VUFORIA_KEY = "AS1gFCv/////AAAAGT+9l7LOzEkhnmqU88TWaZ0FGZEbE+PSk+otNY0JQQ6RVgYi9ZIOKxFkKSKF9rvzVOb6fDI734ntzIl721dBhibt0nhLVeWz98d/pUHZ/FT9pwaMwZssoK+5U7iS5gBmqSY66/R+LXuBlCkjOHXbGwwV5hczxOjOZJlkkK62Jv7Gtt7Va4sPV+1o+MxdZEpr4UXKCV6OJ2r/3OJSW53r0PwTHqpnxTaWuGDuioVbE+2gnDsrG3o5A+hJJqHocRlji2o61cM7BOuhajDdLxD4Rvus9VOh7Jz5j5EDpwLU6HOMONwOmonDpzZBrkukd0vQ/+aNElMzX29sUwebD212KD/Lpv3ozK+H1JHzQHyGRRDi";
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Velocity thing;

    //Mineral detection stuff
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    TFObjectDetector tfod;

    final double ROBOT_FACING_LEFT=-20;
    double robotRotation=ROBOT_FACING_LEFT;

    final int GOLD_MINERAL_LEFT=0;
    final int GOLD_MINERAL_CENTER=1;
    final int GOLD_MINERAL_RIGHT=2;
    final int GOLD_MINERAL_UNKNOWN=999; //UNKNOWN could be any value bigger than 2 or less than 0, just saying
    int goldMineralLocation=GOLD_MINERAL_UNKNOWN;

    ElapsedTime runtime = new ElapsedTime();

    final String GYRO_ANGLE_FILE_NAME="gyroAngle.json";

    ModernRoboticsI2cRangeSensor distanceSensorY;
    Rev2mDistanceSensor distanceSensorX;
    AnalogInput distanceSensorXSonar;

    AnalogInput limitSwitch;

    DcMotor LeftFront;
    DcMotor RightFront;
    DcMotor LeftBack;
    DcMotor RightBack;

    DcMotor hanger;
    DcMotor intake;
    DcMotor arm;
    DcMotor linearSlide;

    RevBlinkinLedDriver lights;

    Servo teamMarker;
    Servo mineralArm;

    ColorSensor colorSensor;

    boolean is11874Bot=false;

    double offsetX=0;
    double offsetY=0;
    double rotationOffset=0;
    final double constantOffsetX=0;
    final double constantOffsetY=0;

    public theColt()
    {
        is11874Bot=false;
    }
    public void INIT(HardwareMap hardwareMap)
    {
        INIT(hardwareMap, false);
    }
    public void INIT(HardwareMap hardwareMap, boolean is11874Bot)
    {
        LeftBack= hardwareMap.get(DcMotor.class, "Left Back");  //set Motors
        RightBack= hardwareMap.get(DcMotor.class, "Right Back");
        LeftFront= hardwareMap.get(DcMotor.class, "Left Front");
        RightFront = hardwareMap.get(DcMotor.class, "Right Front");
        if (!is11874Bot)
        {
            distanceSensorY = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
            distanceSensorX = hardwareMap.get(Rev2mDistanceSensor.class, "range sensor2");
            distanceSensorXSonar = hardwareMap.get(AnalogInput.class, "maxbotix x");
            hanger = hardwareMap.get(DcMotor.class, "hanger");
            arm = hardwareMap.get(DcMotor.class, "arm");
            intake = hardwareMap.get(DcMotor.class, "intake");
            linearSlide= hardwareMap.get(DcMotor.class, "liner slide");
            teamMarker=hardwareMap.get(Servo.class, "team marker");
            mineralArm=hardwareMap.get(Servo.class, "mineral arm");
            lights=hardwareMap.get(RevBlinkinLedDriver.class, "lights");
            limitSwitch=hardwareMap.get(AnalogInput.class, "limit switch");
            colorSensor=hardwareMap.get(ColorSensor.class,"arm color sensor");
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //arm.setTargetPosition(.9);
            zeroLinearSlide();
            zeroArm();
            mineralArm.setPosition(0);
            teamMarker.setPosition(1);
        }
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void lowerRobot()
    {
        hanger.setPower(-1);    // Go fast at first
        runtime.reset();    // Reset the timer
        while (hanger.getCurrentPosition()>-7100 && opModeIsActive() && runtime.seconds()<=6)
        {
            telemetry.addData("hanger motor position", hanger.getCurrentPosition());
            outputTelemetry();
        }
        hanger.setPower(0); // All done. Stop.
    }
    public void raiseRobot()
    {
        hanger.setPower(.5);
        runtime.reset();    // Reset the timer
        while (hanger.getCurrentPosition()<-1000 && opModeIsActive() && runtime.seconds()<=6)
        {
            telemetry.addData("hanger motor position", hanger.getCurrentPosition());
            outputTelemetry();
        }
        hanger.setPower(0);
    }
    public void deployTeamMarker()
    {
        teamMarker.setPosition(.4);
        sleep(.5);
        teamMarker.setPosition(1);
        sleep(.25);
    }
    public void writeFile(String filename, double number)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(number));
            //telemetry.log().add("saved " + number + " in " + filename);
        }
        catch(Exception e)
        {
            // telemetry.log().add("Unable to write " + number + " in " + filename);
        }
    }
    public double readFile(String filename)
    {
        try {
            double output=0;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = Double.parseDouble(ReadWriteFile.readFile(file));
            //telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            //telemetry.log().add("Unable to read " + filename + ", returning 0");
            return 0;
        }
    }
    public void outputTelemetry()
    {
        telemetry.addData("gold mienral location", goldMineralLocation);
        telemetry.addData("default IMU angle", angles.firstAngle);
        telemetry.addData("IMU angle with zero Value", getIMUAngle());
        telemetry.update();
    }
    public void sleep(double seconds)
    {
        runtime.reset();
        while (runtime.seconds()<=seconds && opModeIsActive());
    }
    public void TurnPID(double angle, double seconds)
    {
        TurnPID(angle, seconds, -1);
    }
    public void TurnPID(double angle, double seconds, double motorPowerModifer)
    {
        motorPowerModifer=-Math.abs(motorPowerModifer); //make sure the robot will always go the right direction
        MiniPID miniPID = new MiniPID(.01, 0, .013);
        double angleDiference=angle-getIMUAngle();
        if (Math.abs(angleDiference)>180) //make the angle difference less then 180 to remove unnecessary turning
        {
            angleDiference+=(angleDiference>=0) ? -360 : 360;
        }
        double tempZeroValue=angleZzeroValue;
        angleZzeroValue=0;
        angleZzeroValue=-getIMUAngle();

        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        telemetry.log().add("angle difference " + angleDiference);
        double target=angleDiference;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {

            output = miniPID.getOutput(actual, target);
            if (angle>175 || angle <-175) actual = getIMUAngle(true);
            else actual = getIMUAngle();
            turnLeft(output*motorPowerModifer);
            //if (power>.07 || power <-.07) turnLeft(power/2); else turnLeft(power*1.3);
            // if (power>.5 || power < -.5) turnLeft(power/2);
            // else if (power > .1 || power <-.1) turnLeft(power);
            // else turnLeft(.1*motorPowerModifer);
            outputTelemetry();
        }
        stopRobot();
        angleZzeroValue=tempZeroValue;
    }
    public void TurnPIDandLowerArm(double angle, double seconds, double armLoweredPercent, double armExtensionPercent)
    {
        //make sure the robot will always go the right direction
        MiniPID miniPID = new MiniPID(.01, 0, .013);
        double angleDiference=angle-getIMUAngle();
        if (Math.abs(angleDiference)>180) //make the angle difference less then 180 to remove unnecessary turning
        {
            angleDiference+=(angleDiference>=0) ? -360 : 360;
        }
        double tempZeroValue=angleZzeroValue;
        angleZzeroValue=0;
        angleZzeroValue=-getIMUAngle();

        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        telemetry.log().add("angle difference " + angleDiference);
        double target=angleDiference;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition((int) (armMaxEncoder*armLoweredPercent));
        arm.setPower(.8);

        //MiniPID slidePID = new MiniPID(.10, 0.00, 0);
        double target3 = linearSlideMaxEncoder*armExtensionPercent;
        slidePID.setSetpoint(0);
        slidePID.setSetpoint(target3);
        slidePID.setOutputLimits(1);

        slidePID.setSetpointRange(40);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {

            output = miniPID.getOutput(actual, target);
            if (angle>175 || angle <-175) actual = getIMUAngle(true);
            else actual = getIMUAngle();
            turnLeft(output);
            if (!arm.isBusy()) arm.setPower(0);
            actual=linearSlide.getCurrentPosition();
            output=slidePID.getOutput(actual,target3);
            linearSlide.setPower(output);
            outputTelemetry();
        }
        linearSlide.setPower(0);
        arm.setPower(0);
        stopRobot();
        angleZzeroValue=tempZeroValue;
    }
    void strafeLeft(double power, int target)
    {
         DriveLeft(power);
         zeroEncoders();
         while (Math.abs(RightFront.getCurrentPosition())<Math.abs(target) && opModeIsActive());
         stopRobot();
         
    }
    void intake(double power)
    {
        intake.setPower(-power);
    }
    void outtake(double power)
    {
        intake.setPower(power);
    }
    void stopIntake() {intake.setPower(0);}
    boolean isSilverMineral()
    {
        if (colorSensor.argb()>200)
        {
            telemetry.log().add("DETECTED SILVER MINERAL");
            return true;
        }
        else return false;
    }
    boolean isGoldMineral()
    {
        if (colorSensor.red()>150 && colorSensor.green()>150)
        {
            telemetry.log().add("DETECTED GOLD MINERAL");
            return true;
        }
        else return false;
    }
    double getCurrentPercentArmLowered()
    {
        return arm.getCurrentPosition()/armMaxEncoder;
    }
    double getCurrentPercentArmExtended()
    {
        return linearSlide.getCurrentPosition()/linearSlideMaxEncoder;
    }
    void scanMinerals() {
        tfod.activate();
        try {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    updatedRecognitions =removeSmallArea(updatedRecognitions);
                    if (updatedRecognitions.size()==2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            } else {
                                silverMineral2X = (int) recognition.getTop();
                            }
                        }
                        if (goldMineralX != -1) {
                            if (silverMineral1X>=goldMineralX) goldMineralLocation=GOLD_MINERAL_CENTER;
                            else if (silverMineral1X<goldMineralX) goldMineralLocation=GOLD_MINERAL_RIGHT;
                        } else goldMineralLocation=GOLD_MINERAL_LEFT;
                    }
                    telemetry.addData("gold mineral location", (goldMineralLocation==GOLD_MINERAL_LEFT) ? "LEFT" : (goldMineralLocation==GOLD_MINERAL_CENTER) ? "CENTER" : (goldMineralLocation==GOLD_MINERAL_RIGHT) ? "RIGHT" : "UNKNOWN");
                    telemetry.update();
                    if (goldMineralLocation==GOLD_MINERAL_LEFT) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    else if (goldMineralLocation==GOLD_MINERAL_CENTER) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    else if (goldMineralLocation==GOLD_MINERAL_RIGHT) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
            }
        }
        catch (Exception e)
        {
            telemetry.log().add("Error happened (it's not a big deal)");
        }
    }
    List<Recognition> removeSmallArea(List<Recognition> minerals)
    {
        while (minerals.size()>=3)
        {
            double[] areas = new double[minerals.size()];
            int smallest=0;
            for (int i=0; i < minerals.size(); i++)
            {
                double length = Math.abs(minerals.get(i).getTop()-minerals.get(i).getBottom());
                double width = Math.abs(minerals.get(i).getLeft()-minerals.get(i).getRight());
                areas[i]=length*width;
                if (areas[i]<areas[smallest]) smallest=i;
            }
            minerals.remove(smallest);
        }
        return minerals;
    }
    void initTfod() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            //tfod.activate();
        }

    }
    double getIMUAngle()
    {
        double currentAngle=angles.firstAngle+angleZzeroValue;
        while (currentAngle>180) currentAngle-=360;
        while (currentAngle<-180)currentAngle+=360;
        return currentAngle;
    }
    double getIMUAngle(boolean extendBeyond180)
    {
        double currentAngle=angles.firstAngle+angleZzeroValue;
        if (extendBeyond180) return currentAngle;
        else return getIMUAngle();
    }

    void initIMU()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


    }
    void composeTelemetry() { //called in initIMU()

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });*/
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    void DriveforLength(double feet,double power)
    {
        telemetry.log().add("Drving for length");
        if (power>=0) feet = Math.abs(feet);
        else feet = -Math.abs(feet);
        double gyroAngle=getIMUAngle(); //get the angle of the gyro before the robot takes off
        zeroEncoders();
        driveForward(power);
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        if (drivingDistanceInEncoderTicks>=0) while (RightBack.getCurrentPosition() <=drivingDistanceInEncoderTicks && opModeIsActive())
        {
            telemetry.addData("Encoder ticks to drive (positve)", drivingDistanceInEncoderTicks);
            telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        else while (RightBack.getCurrentPosition() >=drivingDistanceInEncoderTicks && opModeIsActive())
        {
            telemetry.addData("Encoder ticks to drive (negitive)", drivingDistanceInEncoderTicks);
            telemetry.addData("IMU angle", getIMUAngle());
            outputTelemetry();
        }
        stopRobot();
        telemetry.log().add("Done with Drive for length");
        telemetry.log().add("Left Front Encoder Pos:"+ LeftFront.getCurrentPosition());
        telemetry.log().add("Right Front Encoder Pos:"+ RightFront.getCurrentPosition());
        telemetry.log().add("Left Back Encoder Pos:"+ LeftBack.getCurrentPosition());
        telemetry.log().add("Right Back Encoder Pos:"+ RightBack.getCurrentPosition());
        telemetry.log().add("Done with Drive for length");
    }
    void DriveFieldRealtiveDistance(double power, double angleInDegrees, double feet)
    {
        zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        telemetry.log().add("averge encoder" + averageDrivetrainEncoder());
        while (opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            outputTelemetry();
        }
        stopRobot();

    }
    void DriveFieldRealtiveDistanceAndLowerArm(double power, double angleInDegrees, double feet, double armLoweredPercent, double armExtensionPercent)
    {
        zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        //double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        telemetry.log().add("averge encoder" + averageDrivetrainEncoder());
        //set arm infomation
        //MiniPID armPID = new MiniPID(.10, 0.00, 0);
        double target = armMaxEncoder*armLoweredPercent;
        armPID.setSetpoint(0);
        armPID.setSetpoint(target);
        armPID.setOutputLimits(1);

        armPID.setSetpointRange(40);

        //MiniPID slidePID = new MiniPID(.10, 0.00, 0);
        double target2 = linearSlideMaxEncoder*armExtensionPercent;
        slidePID.setSetpoint(0);
        slidePID.setSetpoint(target);
        slidePID.setOutputLimits(1);

        slidePID.setSetpointRange(40);

        double actual=0;
        double output=0;
        while (opModeIsActive() && Math.abs(averageDrivetrainEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        {
            double robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            actual=arm.getCurrentPosition();
            output=armPID.getOutput(actual, target);
            arm.setPower(output);
            actual=linearSlide.getCurrentPosition();
            output=slidePID.getOutput(actual, target2);
            linearSlide.setPower(output);
            outputTelemetry();
            if (!isLimitSwitchPressed()) hanger.setPower(.7);
            else hanger.setPower(0);
            telemetry.addData("Left Front Encoder Pos:", LeftFront.getCurrentPosition());
            telemetry.addData("Right Front Encoder Pos:", RightFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder Pos:", LeftBack.getCurrentPosition());
            telemetry.addData("Right Back Encoder Pos:", RightBack.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0);
        arm.setPower(0);
        hanger.setPower(0);
        stopRobot();

    }
    void driveToPointPIDandLowerArm(double x, double y, double armLoweredPercent, double armExtensionPercent, double time)
    {
        //set arm infomation
        //MiniPID armPID = new MiniPID(.10, 0.00, 0);
        double target = armMaxEncoder*armLoweredPercent;
        armPID.setSetpoint(0);
        armPID.setSetpoint(target);
        armPID.setOutputLimits(1);

        armPID.setSetpointRange(40);

        //MiniPID slidePID = new MiniPID(.10, 0.00, 0);
        double target2 = linearSlideMaxEncoder*armExtensionPercent;
        slidePID.setSetpoint(0);
        slidePID.setSetpoint(target);
        slidePID.setOutputLimits(1);

        slidePID.setSetpointRange(40);

        double actual=0;
        double output=0;
        MiniPID miniPID = new MiniPID(.055, 0.000, 0.04);
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(x);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actualX=0;
        double outputX=0;

        MiniPID miniPIDY = new MiniPID(.10, 0.00, 0.05);
        miniPIDY.setSetpoint(0);
        miniPIDY.setSetpoint(y);
        miniPIDY.setOutputLimits(1);

        miniPIDY.setSetpointRange(40);

        double actualY=0;
        double outputY=0;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<time) {
            actualX = getRobotPositionX();
            outputX = miniPID.getOutput(actualX, x);
            actualY = getRobotPositionY();
            outputY = miniPIDY.getOutput(actualY, y);
            double power = Math.hypot(outputX, outputY);
            if (power>.8) power = .8; //set max power as .8
            double slope = getSlope(x, y, actualX, actualY);
            double angle = Math.toDegrees(Math.atan2(outputX, -outputY));
            DriveFieldRealtiveSimple(power, angle);
            actual=arm.getCurrentPosition();
            output=armPID.getOutput(actual, target);
            arm.setPower(output);
            actual=linearSlide.getCurrentPosition();
            output=slidePID.getOutput(actual, target2);
            linearSlide.setPower(output);
            if (!isLimitSwitchPressed()) hanger.setPower(.7);
            else hanger.setPower(0);
        }
        stopRobot();
        linearSlide.setPower(0);
        arm.setPower(0);
        hanger.setPower(0);
    }
    void lowerArm(double armLoweredPercent, double armExtensionPercent, double time)
    {
        //set arm infomation
        //MiniPID armPID = new MiniPID(.10, 0.00, 0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition((int) (armMaxEncoder*armLoweredPercent));
        arm.setPower(1);

        //MiniPID slidePID = new MiniPID(.10, 0.00, 0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setTargetPosition((int) (linearSlideMaxEncoder*armExtensionPercent));
        linearSlide.setPower(1);
        runtime.reset();
        while (opModeIsActive() && arm.isBusy() && linearSlide.isBusy())
        {
            if (!arm.isBusy()) arm.setPower(0);
            if (!linearSlide.isBusy()) linearSlide.setPower(0);
            telemetry.addData("ARM current Position", arm.getCurrentPosition());
            telemetry.addData("ARM current Position %", getCurrentPercentArmLowered());
            telemetry.addData("LINEAR SLIDE current Position", linearSlide.getCurrentPosition());
            telemetry.addData("LINEAR SLIDE current Position %", getCurrentPercentArmExtended());
            telemetry.update();
        }
        linearSlide.setPower(0);
        arm.setPower(0);
    }
    void driveIntoMineral(double angleParrelToMinerals, int mineralLocation)
    {
        if (tfod!=null)
        {
            tfod.shutdown();
        }
        switchXandY();
        if (goldMineralLocation!=GOLD_MINERAL_CENTER) TurnPID(0, 2);
        if (mineralLocation==GOLD_MINERAL_RIGHT)
        {

            DriveToPointPID(18,38, 2);
            DriveToPointPID(20,5, 2);
        }
        else if (mineralLocation==GOLD_MINERAL_LEFT)
        {
            DriveToPointPID(50,13, 2.5);
            DriveToPointPID(24,5, 2);
        }
        else if (goldMineralLocation==GOLD_MINERAL_CENTER)
        {
            DriveforLength(3.4, -.7);
        }
        telemetry.log().add("third movement done");
    }
    void driveIntoMineralCreator(double angleParrelToMinerals, int mineralLocation)
    {
        if (tfod!=null)
        {
            tfod.shutdown();
        }
        if (goldMineralLocation!=GOLD_MINERAL_CENTER) TurnPID(0, 3);
        if (goldMineralLocation==GOLD_MINERAL_RIGHT)
        {
            DriveToPointPID(24,40,2);
            DriveToPointPID(30,40,2);
            DriveToPointPID(63,3,4);
        }
        else if (goldMineralLocation==GOLD_MINERAL_LEFT)
        {
            DriveToPointPID(46,20, 1.5);
            DriveToPointPID(41,20, 1.5);
            DriveToPointPID(49,15, 1);
            DriveToPointPID(63,3,2);
        }
        else if (goldMineralLocation==GOLD_MINERAL_CENTER)
        {
            DriveforLength(1, -.5);
            DriveforLength(1, .5);
            TurnPID(0,3);
            DriveToPointPID(63,3,3.5);
        }
        sleep(.5);
    }
    void strafeToDistanceXPID(double inch, double gap)
    {
        MiniPID miniPID = new MiniPID(.055, 0.000, 0.04);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<gap) {

            actual = getRobotPositionX();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 90 : 270);
            telemetry.addData("Output", output);
            telemetry.addData("Pos X", getRobotPositionX());
            telemetry.update();
        }

    }
    /*
    void strafeToDistanceYPID(double inch, double gap)
    {
        MiniPID miniPID = new MiniPID(.10, 0.00, 0.05);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<gap) {

            actual = getRobotPositionY();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 180 : 0);
            telemetry.addData("Output", output);
            telemetry.addData("Pos Y", getRobotPositionY());
            telemetry.update();
        }
        stopRobot();
    }
    */
    void strafeToDistanceYPID(double inch, double gap) { strafeToDistanceYPID(inch,gap,0);}
    void strafeToDistanceYPID(double inch, double gap, double offset)
    {
        MiniPID miniPID = new MiniPID(.10, 0.00, 0.05);
        double target = inch;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actual=0;
        double output=0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<gap) {

            actual = getRobotPositionY();
            output = miniPID.getOutput(actual, target);
            DriveFieldRealtiveSimple(output, (output>=0) ? 180+offset : 0+offset);
            telemetry.addData("Output", output);
            telemetry.addData("Pos Y", getRobotPositionY());
            telemetry.update();
        }
        stopRobot();
    }
    public void switchXandY()
    {
        //ModernRoboticsI2cRangeSensor temp =  distanceSensorY;
        //distanceSensorY= distanceSensorX;
        //distanceSensorX=temp;
        if (rotationOffset==90) setOffset(0,0,0);
        else setOffset(0,0,90);
    }
    /*public void DriveToPointPID(double x, double y, double seconds)
    {
        MiniPID miniPID = new MiniPID(.055, 0.000, 0.04);
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(x);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actualX=0;
        double outputX=0;

        MiniPID miniPIDY = new MiniPID(.10, 0.00, 0.05);
        miniPIDY.setSetpoint(0);
        miniPIDY.setSetpoint(y);
        miniPIDY.setOutputLimits(1);

        miniPIDY.setSetpointRange(40);

        double actualY=0;
        double outputY=0;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<seconds) {
            actualX = getRobotPositionX();
            outputX = miniPID.getOutput(actualX, x);
            actualY = getRobotPositionY();
            outputY = miniPIDY.getOutput(actualY, y);
            double power = Math.hypot(outputX, outputY);
            if (power>.8) power = .8; //set max power as .8
            double slope = getSlope(x, y, actualX, actualY);
            double angle = Math.toDegrees(Math.atan2(outputX, -outputY));
            DriveFieldRealtiveSimple(power, angle);
            if (Math.abs(getIMUAngle())>=5)
            {
                TurnPID(0,.5);
                seconds++;
            }
        }
        stopRobot();
    }
    */
    public void DriveToPointPID(double x, double y, double seconds) { DriveToPointPID(x,y,seconds,0);}
    public void DriveToPointPID(double x, double y, double seconds,double offset)
    {
        MiniPID miniPID = new MiniPID(.055, 0.000, 0.04);
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(x);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actualX=0;
        double outputX=0;

        MiniPID miniPIDY = new MiniPID(.10, 0.00, 0.05);
        miniPIDY.setSetpoint(0);
        miniPIDY.setSetpoint(y);
        miniPIDY.setOutputLimits(1);

        miniPIDY.setSetpointRange(40);

        double actualY=0;
        double outputY=0;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<seconds) {
            actualX = getRobotPositionX();
            outputX = miniPID.getOutput(actualX, x);
            actualY = getRobotPositionY();
            outputY = miniPIDY.getOutput(actualY, y);
            double power = Math.hypot(outputX, outputY);
            if (power>.8) power = .8; //set max power as .8
            double slope = getSlope(x, y, actualX, actualY);
            double angle = Math.toDegrees(Math.atan2(outputX, -outputY)) + offset;
            DriveFieldRealtiveSimple(power, angle);
            if (Math.abs(getIMUAngle())>=5)
            {
                TurnPID(0,.5);
                seconds++;
            }
        }
        stopRobot();
    }
    public void DriveFieldRealtiveSimple(double power, double angle)
    {
        power=Math.abs(power);
        double angleInRadians =Math.toRadians(angle)-Math.PI / 4;
        double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        LeftFront.setPower(power * Math.cos(angleInRadians-robotAngle) + rightX);
        RightFront.setPower(power * Math.sin(angleInRadians-robotAngle) - rightX);
        LeftBack.setPower(power * Math.sin(angleInRadians-robotAngle) + rightX);
        RightBack.setPower(power * Math.cos(angleInRadians-robotAngle) - rightX);

    }
    public void setOffset(double RobotOffsetX, double RobotOffsetY)
    {
        offsetX=RobotOffsetX+constantOffsetX;
        offsetY=RobotOffsetY+constantOffsetY;
    }
    public void setOffset(double RobotOffsetX, double RobotOffsetY, double RobotRotationOffset)
    {
        setOffset(RobotOffsetX, RobotOffsetY);
        rotationOffset=RobotRotationOffset;
    }
    public double getRobotPositionXSonar()
    {
        double voltage=distanceSensorXSonar.getVoltage();
        double distance=6+(voltage / 0.006);
//        telemetry.log().add("voltage = %d, distance = %d", voltage, distance);
        return distance;
    }
    public double getRobotPositionXRev2m()
    {
        double distance=(distanceSensorX.getDistance(DistanceUnit.INCH));
        if (distance==0)
        {
            stopRobot();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
            distanceSensorX = hardwareMap.get(Rev2mDistanceSensor.class, "range sensor2");
            distance=(distanceSensorX.getDistance(DistanceUnit.INCH));
            telemetry.log().add("DISTANCE SENSOR X BROKE!!!!!!!");
            sleep(1000);
        }
        if (distance>144)
        {
            telemetry.log().add("overshot X distance");
            return lastPosX;
        }
        lastPosX=distance;
        return distance;
    }
    public double getRobotPositionX()
    {
        double distanceSonar = getRobotPositionXSonar();
        double distance2m = getRobotPositionXRev2m();
        double distanceBest = (distanceSonar+distance2m)/2.0;   // Assume the average is best

        if (distanceSonar < 13.0) {  // Sonar only works until 6 inches so prefer Rev2m below that
            distanceBest = distance2m;
//            telemetry.log().add("using Rev 2m distance");
        }
        if (distanceSonar > 35.0) { // Beyond X inches the Rev 2m doesn't work so prefer sonar
            distanceBest = distanceSonar;
        }
        if (distance2m > 130) {
            distanceBest = distanceSonar;   // If the Rev returns over X, ignore it
        }
        return distanceBest;
    }
    public double getRobotPositionY()
    {
        double distance=(distanceSensorY.getDistance(DistanceUnit.INCH));
        if (Double.isNaN(distance))
        {
            telemetry.log().add("DISTANCE Y VALUE WAS NaN");
            return 2.5;
        }
        if (distance==0)
        {
            stopRobot();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
            distanceSensorY = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
            distance=(distanceSensorY.getDistance(DistanceUnit.INCH));
            telemetry.log().add("DISTANCE SENSOR Y BROKE!!!!!!!");
            sleep(1d);
        }
        if (distance>144)
        {
            telemetry.log().add("overshot Y distance");
            return lastPosY;
        }
        lastPosY=distance;
        return distance;
    }
    public boolean isLimitSwitchPressed()
    {
        if (limitSwitch.getVoltage()>1.5) return true;
        else return false;
    }
    double getSlope(double x1, double y1, double x2, double y2)
    {
        return (y1-y2)/(x1-x2);
    }
    double averageDrivetrainEncoder()
    {
        double motorposition=0;
        motorposition+=LeftBack.getCurrentPosition();
        motorposition+=LeftFront.getCurrentPosition();
        motorposition+=RightBack.getCurrentPosition();
        motorposition+=RightFront.getCurrentPosition();
        if (Double.isNaN(motorposition/4)) return 0;
        else return motorposition/4;
    }
    void zeroEncoders()
    {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void zeroHanger()
    {
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void zeroArm()
    {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void zeroLinearSlide()
    {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveForward(double power)
    {
        LeftFront.setPower(power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(power);
    }
    public void turnLeft(double power) {
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }
    public void DriveLeft(double power) {
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }
    public void stopRobot()
    {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }
    public void runOpMode() {}
}
