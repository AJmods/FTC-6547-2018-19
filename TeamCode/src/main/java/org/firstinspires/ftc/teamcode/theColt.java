package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import java.io.File;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Drew from 11874 on 10/17/2018.
 */

@Disabled
public class theColt extends LinearOpMode{

    final double encoderTicksPerRotation=510;
    final double circumferenceOfWheel=12.566370614359172;
    //Vuforia Images (these are just here to make the code easier to read
    final String VuforiaImage_MARS = "Front-Mars";
    final String VuforiaImage_ROVER = "Blue-Rover";
    final String VuforiaImage_SPACE = "Back-Space";
    final String VuforiaImage_MOON = "Red-Moon";
    final String VuforiaImage_UNKNOWN = "UNKNOWN";
    //Stores the Value of the Vuforia Image
    String VuforiaImage = VuforiaImage_UNKNOWN;
    //Offset caused by Vufoira Image, keep this at 0
    double VuforiaOffsetX=0;
    double VuforiaOffsetY=0;
    //Angle the robot phone is facing the VuforiaImage
    double VuforiaAngleY;
    //Distace away the robot phone is from the VuforiaImage
    double VuforiaDistance;
    //Stores the postion of the robot phone on the robot (IN FEET) (18 inches is 1.5 feet), entered in code'
    final double robotPhonePosOnRobotX=1.166666;
    final double robotPhonePosOnRobotY=.916666;
    //angle the robot phone is positioned on the robot.  0 degrees is facing forwards
    final double robotPhoneAngleOnRobot=-11.5; //0 degrees is the robot phone camera parallel with the LEFT of the robot, 90 is the camera is parallel with the FRONT of the robot, 180 is the RIGHT, 270 is the BACK.
    //skip the Y axis because these variables use the field coordinate plane, the Y axis is an unnecessary extra dimension (up and down)
    double VuforiaX=0;
    double VuforiaY=0;  //Z axis for Vuforia, but this is treated like the Y axis

    //position of robot phone
    double robotPhoneX=0;
    double robotPhoneY=0;
    //position of robot
    double robotTopLeftX;
    double robotTopRightX;
    double robotBottomLeftX;
    double robotBottomRightX;
    double robotTopLeftY;
    double robotTopRightY;
    double robotBottomLeftY;
    double robotBottomRightY;
    double robotCenterX;
    double robotCenterY;

    double angleZ;
    double angleZzeroValue=0;

    static double lastPosX=40;
    static double lastPosY=40;

    final String BOTTOM_RED = "creater red";
    final String TOP_RED = "depot red";
    final String TOP_BLUE = "creater blue";
    final String BOTTOM_BLUE = "depot blue";
    String robotFieldSection;

    String direction;
    final String DIRECTION_UPRIGHT="upright", DIRECTION_DOWNRIGHT="downright", DIRECTION_UPLEFT="upleft", DIRECTION_DOWNLEFT="downleft"; //directions for later use

    static final String VUFORIA_KEY = "AS1gFCv/////AAAAGT+9l7LOzEkhnmqU88TWaZ0FGZEbE+PSk+otNY0JQQ6RVgYi9ZIOKxFkKSKF9rvzVOb6fDI734ntzIl721dBhibt0nhLVeWz98d/pUHZ/FT9pwaMwZssoK+5U7iS5gBmqSY66/R+LXuBlCkjOHXbGwwV5hczxOjOZJlkkK62Jv7Gtt7Va4sPV+1o+MxdZEpr4UXKCV6OJ2r/3OJSW53r0PwTHqpnxTaWuGDuioVbE+2gnDsrG3o5A+hJJqHocRlji2o61cM7BOuhajDdLxD4Rvus9VOh7Jz5j5EDpwLU6HOMONwOmonDpzZBrkukd0vQ/+aNElMzX29sUwebD212KD/Lpv3ozK+H1JHzQHyGRRDi";

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    OpenGLMatrix lastLocation = null;
    boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables;

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


    final double ROBOT_FACING_CENTER=0;
    final double ROBOT_FACING_RIGHT=20;
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

    boolean is11874Bot=false;

    static boolean encoderMotorsLoaded=false;

    List<DcMotor> encoderMotors=new ArrayList<DcMotor>();

    double[] terms;
    double equationResolution =.02;
    double offsetX=0;
    double offsetY=0;
    double rotationOffset=0;
    final double constantOffsetX=0;
    final double constantOffsetY=0;
    
    double XDifference=0;
    double YDifference=0;
    boolean isRobotInX=false;
    boolean isRobotInY=false;
    
    double lastPosXrev=0;
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
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
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
        /*hanger.setPower(.5);
        runtime.reset();    // Reset the timer
        while (hanger.getCurrentPosition()<-1000 && opModeIsActive() && runtime.seconds()<=6)
        {
            telemetry.addData("hanger motor position", hanger.getCurrentPosition());
            outputTelemetry();
        }
        hanger.setPower(0);
        */
        hanger.setPower(.5);
        while (opModeIsActive() && !isLimitSwitchPressed())
        {
            telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
            telemetry.update();
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
    public void getToVuforiaTarget()
    {
        //driveNoVuforia(2.5,-.5);
        sleep(.5);
        //DriveLeft(.2);
        runtime.reset();
        while (!isVuforiaImageThere() && runtime.seconds() < 3 && opModeIsActive())
        {
            outputTelemetry();
        }
        stopRobot();

    }
    public boolean isVuforiaImageThere()
    {
        scanVufoira();
        if (targetVisible) return true;
        else return false;
    }
    public void outputTelemetry()
    {
        // telemetry.addData("Section", robotFieldSection);
        // telemetry.addData("Visible Target", VuforiaImage);
        // telemetry.addData("Robot Phone Position (" + round(robotPhoneX,2) + "," + round(robotPhoneY,2) + ")","");
        // telemetry.addData("Left Front current Pos", LeftFront.getCurrentPosition());
        // telemetry.addData("Left Back current Pos", LeftBack.getCurrentPosition());
        // telemetry.addData("Right Front current Pos", RightFront.getCurrentPosition());
        // telemetry.addData("Right Back current Pos", RightBack.getCurrentPosition());
        // telemetry.addData("Center Robot Position (" + robotCenterX + "," + robotCenterY + ")","");
        // telemetry.addData("Top Left Robot Position (" + robotTopLeftX + "," + robotTopLeftY + ")","");
        // telemetry.addData("Top Right Robot Position (" + robotTopLeftX + "," + robotTopLeftY + ")","");
        // telemetry.addData("Bottom Left Robot Position (" + robotBottomLeftX + "," + robotBottomLeftY + ")","");
        // telemetry.addData("Bottom Right Robot Position (" + robotBottomRightX + "," + robotBottomRightY + ")","");
        // telemetry.addData("Vuforia Angle Y", VuforiaAngleY);
        // telemetry.addData("Zero Value", angleZzeroValue);
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
    public void simpleDrive(double destinationX, double destinationY, double angleToTurn, double power)
    {
        sleep(.5);
        Turn(angleToTurn, .3, 10);
        sleep(.5);
        Turn(angleToTurn, .1, 7);
        sleep(.5);
        Turn(angleToTurn, .1, 5);
        sleep(.5);
        Turn(angleToTurn, .1, 3);
        sleep(.5);
        Turn(angleToTurn, .06, 2);
        sleep(.5);
        Turn(angleToTurn, .05, 2);
        sleep(.5);
        Turn(angleToTurn, .05, 1);
        sleep(.5);
        if (getIMUAngle()>=0 && getIMUAngle()<90) direction = DIRECTION_UPRIGHT;
        else if (getIMUAngle()>=90 && getIMUAngle()<=180) direction = DIRECTION_UPLEFT;
        else if (getIMUAngle()<0 && getIMUAngle()>-90) direction=DIRECTION_DOWNRIGHT;
        else if (getIMUAngle()<=-90 && getIMUAngle()>=-180) direction =DIRECTION_DOWNLEFT;
        double drivingDistance = Math.sqrt(Math.pow(destinationX - robotPhoneX, 2) + Math.pow(destinationY - robotPhoneY, 2)); //distance formula
        DriveforLength(drivingDistance, power);
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
    public void Turn(double targetAngle, double motorPower, double leeway)
    {
        if (getIMUAngle()>targetAngle-1 && getIMUAngle()<targetAngle+1) return;
        leeway=Math.abs(leeway); //make sure the leeway is positive
        double angleDiference=targetAngle-getIMUAngle();
        if (targetAngle>=180-leeway) targetAngle-=leeway;
        else if (targetAngle<=-180+leeway) targetAngle+=leeway;
        if (Math.abs(angleDiference)>180) //make the angle difference less then 180 to remove unnecessary turning
        {
            angleDiference+=(angleDiference>=0) ? -360 : 360;
        }
        if (angleDiference>0) motorPower*=-1; //turn right
        turnLeft(motorPower);
        boolean keepTurning=false;
        if (angleDiference>0 && targetAngle<0 && getIMUAngle()>0) keepTurning=true;
        else if (angleDiference<0 && targetAngle>0 && getIMUAngle()<0) keepTurning=true;

        while ((keepTurning && opModeIsActive()) || (opModeIsActive() && (angleDiference>=0) ? getIMUAngle() <= targetAngle-leeway : getIMUAngle() >= targetAngle+leeway))
        {
            if (angleDiference>0 && targetAngle<0 && getIMUAngle()<0) keepTurning=false;
            else if (angleDiference<0 && targetAngle>0 && getIMUAngle()>0) keepTurning=false;
            telemetry.addData("IMU angle", getIMUAngle());
            telemetry.addData("target angle",targetAngle);
            telemetry.addData("angle difference", angleDiference);
            telemetry.addData("leeway", leeway);
            outputTelemetry();
        }
        stopRobot();
    }
    public void calculatePositionWithVufoira()
    {
        //get the Vuforia offset
        if (VuforiaImage.equals(VuforiaImage_MARS))
        {
            VuforiaOffsetX=6;
            VuforiaOffsetY=0;
        }
        else if (VuforiaImage.equals(VuforiaImage_ROVER))
        {
            VuforiaOffsetX=0;
            VuforiaOffsetY=6;
        }
        else if (VuforiaImage.equals(VuforiaImage_SPACE))
        {
            VuforiaOffsetX=6;
            VuforiaOffsetY=12;
        }
        else if (VuforiaImage.equals(VuforiaImage_MOON))
        {
            VuforiaOffsetX=12;
            VuforiaOffsetY=6;
        }

        robotPhoneX = VuforiaX + VuforiaOffsetX;
        robotPhoneY = VuforiaY + VuforiaOffsetY;


        setRobotPostions();

        //output robot position
        if (robotPhoneX >=6 && robotPhoneY <=6) robotFieldSection = BOTTOM_RED;
        else if (robotPhoneX >=6 && robotPhoneY >=6) robotFieldSection = TOP_RED;
        else if (robotPhoneX <=6 && robotPhoneY <=6) robotFieldSection = BOTTOM_BLUE;
        else if (robotPhoneX <=6 && robotPhoneY >=6) robotFieldSection = TOP_BLUE;

    }
    void offSetIMUwithVuforia()
    {
        //set the offset for zero, I want zero degrees to be the front of the robot facing the back wall.
        initIMU();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        if (VuforiaImage.equals(VuforiaImage_SPACE)) angleZzeroValue = -VuforiaAngleY;
        else if (VuforiaImage.equals(VuforiaImage_MARS)) angleZzeroValue = VuforiaAngleY+90;
        else if (VuforiaImage.equals(VuforiaImage_MOON)) angleZzeroValue = -VuforiaAngleY-90;
        else if (VuforiaImage.equals(VuforiaImage_ROVER)) angleZzeroValue = VuforiaAngleY; //if that doesn't work, try angleZzeroValue = (VuforiaAngleY >=0) ? VuforiaAngleY+180 : VuforiaAngleY-180;
    }
    void setRobotPostions() //takes the robot phone position and calulates the top left, top right, bottom right, bottom left, and center positions of the robot
    {
        //calculate the robot positions, after this we will not need to use robotPhoneX and robotPhoneY anymore in the program
        double angle1=getIMUAngle(); //the angle the robot is facing, chnage to getIMUangle();
        while (angle1 >= 90) angle1-=90; //get the angle to be less than 90, for easier calculations
        while (angle1 <0) angle1+=90; //if the angle is negative, make it positive by adding 90 until the angle is positive
        //NEW setRobotPostions()
        //1.5 means 18 inches, and were are assuming the robot is 18x18 inches
        //To calculate the rotated points, we set the current points as if the robot's center point was (0,0) and the robot is at 0 degrees
        robotBottomLeftX=-.75;
        robotBottomLeftY=-.75;
        robotBottomRightX=.75;
        robotBottomRightY=-.75;
        robotTopLeftX=-.75;
        robotTopLeftY=.75;
        robotTopRightX=.75;
        robotTopRightY=.75;
        //now let's rotate the points!!!!!!!  Also the temp variable is there to store X values before they are rotated the calculate the rotated Y value.
        double temp = robotTopLeftX;
        robotTopLeftX = getRotationX(robotTopLeftX,robotTopLeftY,angle1);
        robotTopLeftY = getRotationY(temp, robotTopLeftY, angle1);
        temp = robotTopRightX;
        robotTopRightX = getRotationX(robotTopRightX, robotTopRightY, angle1);
        robotTopRightY = getRotationY(temp, robotTopRightY,angle1);
        temp = robotBottomLeftX;
        robotBottomLeftX = getRotationX(robotBottomLeftX, robotBottomLeftY, angle1);
        robotBottomLeftY = getRotationY(temp, robotBottomLeftY, angle1);
        temp = robotBottomRightX;
        robotBottomRightX = getRotationX(robotBottomRightX, robotBottomRightY,angle1);
        robotBottomRightY = getRotationY(temp, robotBottomRightY, angle1);
        //create an offset to find out the distance between the robot phone and the robot phones location, and the offset will soon be added to the robot postions to get the robot's corner's position on the field.
        double offsetX = robotPhoneX-robotPhonePosOnRobotX;
        double offsetY = robotPhoneY-robotPhonePosOnRobotY;
        //add the offset back to the coordinates
        robotTopLeftX+=offsetX;
        robotTopLeftY+=offsetY;
        robotTopRightX+=offsetX;
        robotTopRightY+=offsetY;
        robotBottomRightX+=offsetX;
        robotBottomRightY+=offsetY;
        robotBottomLeftX+=offsetX;
        robotBottomLeftY+=offsetY;
        robotCenterX = offsetX; //becuase is the center of the robot, we can just make the center positons the offset insted of doing 0+offset
        robotCenterY = offsetY;
        //and done, the robot postions are now rotated
        //OLD setRobotPostions()
        /*//to do the rotations, we first  calculate the robot phones's rotated position on the robot from an equation from the internet
        //let's calculate the robot phone positions.  First we get the not rotated points and offset them so 0,0 is the middle, and the equation I'm using assumes the coordinate plain is being rotated from 0,0
        //The first we get the non rotated point, then we offset the lines so 0,0 is the middle
        robotTopLeftX = robotPhoneX + (1.5-robotPhonePosOnRobotX);
        robotTopLeftY = robotPhoneY + (1.5-robotPhonePosOnRobotY);
        telemetry.addData("robot Top left no rotation", "(" + robotTopLeftX + "," + robotTopLeftY + ")" );
        robotTopRightX = robotPhoneX + robotPhonePosOnRobotX;
        robotTopRightY = robotPhoneY + (1.5-robotPhonePosOnRobotY);
        robotBottomLeftX = robotPhoneX + (1.5-robotPhonePosOnRobotX);
        robotBottomLeftY = robotPhoneY - robotPhonePosOnRobotY;
        robotBottomRightX = robotPhoneX + robotPhonePosOnRobotX;
        robotBottomRightY = robotPhoneY - robotPhonePosOnRobotY;
        robotCenterX = (robotTopLeftX + robotTopRightX + robotBottomLeftX + robotBottomRightX)/4; //average the points for the center robot positions, becuase that's the simpleist way to do it
        robotCenterY = (robotTopLeftY + robotTopRightY + robotBottomLeftY + robotBottomRightY)/4;
        if (angle1==0) return; //exit the method because 0 is a weird number for these equations, and if the angle is 0, we don't need to do all these complex calculations,so the method ends early
        //offset the lines so the center is 0,0
        //we don't rotate the center because the center will always be the same point, no matter the the rotation
        double offsetX = robotTopLeftX-.75;
        double offsetY = robotTopLeftY-.75;
        robotTopLeftX-=offsetX;
        robotTopLeftY-=offsetY;
        robotTopRightX-=offsetX;
        robotTopRightY-=offsetY;
        robotBottomRightX-=offsetX;
        robotBottomRightY-=offsetY;
        robotBottomLeftX-=offsetX;
        robotBottomLeftY-=offsetY;
        //now let's rotate them!!!!!!!
        double temp = robotTopLeftX;
        robotTopLeftX = getRotationX(robotTopLeftX,robotTopLeftY,angle1);
        robotTopLeftY = getRotationY(temp, robotTopLeftY, angle1);
        temp = robotTopRightX;
        robotTopRightX = getRotationX(robotTopRightX, robotTopRightY, angle1);
        robotTopRightY = getRotationY(temp, robotTopRightY,angle1);
        temp = robotBottomLeftX;
        robotBottomLeftX = getRotationX(robotBottomLeftX, robotBottomLeftY, angle1);
        robotBottomLeftY = getRotationY(temp, robotBottomLeftY, angle1);
        temp = robotBottomRightX;
        robotBottomRightX = getRotationX(robotBottomRightX, robotBottomRightY,angle1);
        robotBottomRightY = getRotationY(temp, robotBottomRightY, angle1);
        //add the offset back to the coordinates
        robotTopLeftX+=offsetX;
        robotTopLeftY+=offsetY;
        robotTopRightX+=offsetX;
        robotTopRightY+=offsetY;
        robotBottomRightX+=offsetX;
        robotBottomRightY+=offsetY;
        robotBottomLeftX+=offsetX;
        robotBottomLeftY+=offsetY;
        //and done, the robot postions are now rotated
        */
    }
    double getRotationX(double x, double y,double angle)
    {
        return (x * Math.cos(Math.toRadians(angle))) + (y * Math.sin(Math.toRadians(angle)));
    }
    double getRotationY(double x, double y,double angle)
    {
        return -(x * Math.sin(Math.toRadians(angle))) + (y * Math.cos(Math.toRadians(angle)));
    }
    double round(double number, int decimalPlaces)
    {
        number*=Math.pow(10, decimalPlaces); //move the number across some deciaml places
        number = Math.round(number);
        number/=Math.pow(10, decimalPlaces);
        return number;
    }
    public void scanVufoira()
    {
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                VuforiaImage=trackable.getName();

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(2) / mmPerInch, translation.get(1) / mmPerInch);
            if (VuforiaImage.equals(VuforiaImage_MARS)) //the front Vuforia target
            {
                VuforiaX = -inchesToFeet(translation.get(0) / mmPerInch); //negative X to keep the values correct
                VuforiaY = inchesToFeet(translation.get(2) / mmPerInch);
            }
            else if (VuforiaImage.equals(VuforiaImage_SPACE)) //the top Vuforia target
            {
                VuforiaX = inchesToFeet(translation.get(0) / mmPerInch);
                VuforiaY = -inchesToFeet(translation.get(2) / mmPerInch); //make Y negative to prevent the Y position to go over 12, the maximum value for the coordinate system
            }
            else if (VuforiaImage.equals(VuforiaImage_MOON)) //the red Vuforia target
            {
                VuforiaX = -inchesToFeet(translation.get(2) / mmPerInch); //swaps the X and Y values so the side images are read correctly.  Both values are also negative to prevent the coordinates from going over 12, the maximum value for the coordinate system
                VuforiaY = -inchesToFeet(translation.get(0) / mmPerInch);
            }
            else if (VuforiaImage.equals(VuforiaImage_ROVER)) //the blue Vuforia target
            {
                VuforiaX = inchesToFeet(translation.get(2) / mmPerInch); //swaps the X and Y values so the side images are read correctly.
                VuforiaY = inchesToFeet(translation.get(0) / mmPerInch);
            }
            //VuforiaDistance = inchesToFeet(Math.sqrt(Math.pow(translation.get(0) / mmPerInch,2) + Math.pow(translation.get(2) / mmPerInch,2) + Math.pow(translation.get(1) / mmPerInch,2))); //swapped Z and Y so we can see the X Y relationship
            //telemetry.addData("Distance = ", VuforiaDistance);


            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            VuforiaAngleY = rotation.secondAngle-robotPhoneAngleOnRobot; //offset the Vuforia angle by the real angle on the robot.  No idea if this will works.

            telemetry.addData("Visible Target", VuforiaImage);
        }
        else {
            telemetry.addData("Visible Target", "UNKNOWN");

        }
        outputTelemetry();
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
    void knockOverMineral()
    {
        if (tfod != null) {
            tfod.shutdown();
        }
        //aline the robot with the minerals (probably won't need this, this code is more "just in case").
        if (goldMineralLocation==GOLD_MINERAL_CENTER && robotRotation!=ROBOT_FACING_CENTER) setRobotRotation(ROBOT_FACING_CENTER);
        else if (goldMineralLocation==GOLD_MINERAL_LEFT && robotRotation!=ROBOT_FACING_LEFT) setRobotRotation(ROBOT_FACING_LEFT);
        else if (goldMineralLocation==GOLD_MINERAL_RIGHT && robotRotation!=ROBOT_FACING_RIGHT) setRobotRotation(ROBOT_FACING_RIGHT);
        DriveforLength(1,-.2);
        sleep(.5);
        DriveforLength(1,.2);
    }
    void setRobotRotation(double rotation) //only used for scaning the minerals
    {
        robotRotation = rotation;
        Turn(-135+rotation,.1,3);
    }
    public void InitVufoira()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        /*
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName(VuforiaImage_ROVER);
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName(VuforiaImage_MOON);
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName(VuforiaImage_MARS);
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName(VuforiaImage_SPACE);
        // For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 0 : 0, 0, 0));
        //  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        targetsRoverRuckus.activate(); //best to move to after WaitForStart();
        */
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
    double inchesToFeet(double inches)
    {
        return inches/12;
    }
    void DriveToPoint(double destinationX, double destinationY) //method is not ready to be used
    {
        //make a straight line to the destination
        double slopeOfLine = ((robotPhoneY - destinationY)/(robotPhoneX-destinationX));
        double degreesofSlope = Math.toDegrees(Math.atan(slopeOfLine));
        double angleToTurnAt=degreesofSlope;
        if (destinationY>robotCenterY && degreesofSlope>0) angleToTurnAt=-180+degreesofSlope; //goes down, and the slope is positive
        else if (destinationY<robotCenterY && degreesofSlope<0) angleToTurnAt=180+degreesofSlope; //goes up, slope is negative.
        if (angleToTurnAt>=0 && angleToTurnAt<90) direction = DIRECTION_DOWNRIGHT;
        else if (angleToTurnAt>=90 && angleToTurnAt<=180) direction = DIRECTION_DOWNLEFT;
        else if (angleToTurnAt<0 && angleToTurnAt>-90) direction=DIRECTION_UPLEFT;
        else if (angleToTurnAt<=-90 && angleToTurnAt>=-180) direction =DIRECTION_UPRIGHT;
        TurnPID(angleToTurnAt,6);
        double drivingDistance=Math.sqrt(Math.pow(destinationX-robotPhoneX,2)+Math.pow(destinationY-robotPhoneY,2)); //distance formula
        DriveforLength(drivingDistance,-.5);

    }
    String invertDirection(String directionToInvert)
    {
        if (directionToInvert.equals(DIRECTION_UPRIGHT)) return DIRECTION_DOWNLEFT;
        else if (directionToInvert.equals(DIRECTION_UPLEFT)) return DIRECTION_DOWNRIGHT;
        else if (directionToInvert.equals(DIRECTION_DOWNRIGHT)) return DIRECTION_UPLEFT;
        else if (directionToInvert.equals(DIRECTION_DOWNLEFT)) return DIRECTION_UPRIGHT;
        else return directionToInvert;
    }
    void DriveforLength(double feet,double power)
    {
        telemetry.log().add("Drving for length");
        if (power>=0) feet = Math.abs(feet);
        else feet = -Math.abs(feet);
        double gyroAngle=getIMUAngle(); //get the angle of the gyro before the robot takes off
        if (gyroAngle>=0 && gyroAngle<90) direction = DIRECTION_DOWNRIGHT;
        else if (gyroAngle>=90 && gyroAngle<=180) direction = DIRECTION_DOWNLEFT;
        else if (gyroAngle<0 && gyroAngle>-90) direction=DIRECTION_UPLEFT;
        else if (gyroAngle<=-90 && gyroAngle>=-180) direction =DIRECTION_UPRIGHT;
        if (power<0) direction = invertDirection(direction);
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
        //calulate the location of where the robot went
        double encoderDistance=RightBack.getCurrentPosition();
        double feetTraveled=inchesToFeet(encoderDistance/encodersPerInch);
        double distanceY=0;
        double distanceX=0;
        telemetry.log().add("Direction = " + direction);
        if (true) { //!=90 and !=-90 won't work for some reason and Math.tan() will not work if the angle is 90 or -90, so this is just a safety measure.
            double slopeDrivin = Math.tan(gyroAngle);
            distanceY=feetTraveled*Math.sin(Math.toRadians(gyroAngle));
            distanceX=feetTraveled*Math.cos(Math.toRadians(gyroAngle));
            telemetry.log().add("direction is " + direction);
            if (direction.equals(DIRECTION_UPRIGHT))
            {
                distanceX=-Math.abs(distanceX);
                distanceY=-Math.abs(distanceY);
            }
            else if (direction.equals(DIRECTION_UPLEFT))
            {
                distanceX=Math.abs(distanceX);
                distanceY=-Math.abs(distanceY);
            }
            else if (direction.equals(DIRECTION_DOWNRIGHT))
            {
                distanceX=-Math.abs(distanceX);
                distanceY=Math.abs(distanceY);
            }
            else if (direction.equals(DIRECTION_DOWNLEFT))
            {
                distanceX=Math.abs(distanceX);
                distanceY=Math.abs(distanceY);
            }
        }
        telemetry.log().add("DistanceX = " + distanceX);
        telemetry.log().add("DistanceY = " + distanceY);
        robotPhoneX+=distanceX;
        robotPhoneY+=distanceY;
        setRobotPostions();
         telemetry.log().add("Done with Drive for length");
    }
    boolean isDistanceTarget(double inches, double distanceSensorAngle)
    {
        while (distanceSensorAngle>90) distanceSensorAngle-=90;
        while (distanceSensorAngle<-90) distanceSensorAngle+=90;

        double currentDistance=0;
        double targetDistance = getAngledDistance(inches, distanceSensorAngle);
        if (currentDistance<targetDistance) return true;
        else return false;
    }
    double getAngledDistance(double distance, double angle)
    {
        return distance/Math.cos(Math.toRadians(angle));
    }
    void driveRealtiveDistance(double power, double distance, double angle)
    {
        double currentDistance = getAngledDistance(0, angle);
        double targetDistance = getAngledDistance(distance, angle);
        double angleOffset = Math.toRadians(currentDistance-targetDistance)*5;
        double robotAngle = Math.toRadians(angle);
        double rightX = 0;
        LeftFront.setPower(power * Math.cos(angleOffset-robotAngle) + rightX);
        RightFront.setPower(power * Math.sin(angleOffset-robotAngle) - rightX);
        LeftBack.setPower(power* Math.sin(angleOffset-robotAngle) + rightX);
        RightBack.setPower(power * Math.cos(angleOffset-robotAngle) - rightX);
    }
    void DriveFieldRealtive(double power, double feet)
    {
        DriveFieldRealtive(power, getIMUAngle(), feet);
    }
    void DriveFieldRealtive(double power, double angleInDegrees, double feet)
    {
        //zeroEncoders();
        double inches = feet*12;
        double encodersPerInch = encoderTicksPerRotation/circumferenceOfWheel;
        double drivingDistanceInEncoderTicks = encodersPerInch*inches;
        double speed = power;
        double desiredAngle =Math.toRadians(angleInDegrees)-Math.PI / 4;
        double robotAngle = Math.toRadians(getIMUAngle());
        double rightX = 0;
        LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
        RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
        LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
        RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
        //telemetry.log().add("averge encoder" + averageEncoder());
        //while (opModeIsActive() && Math.abs(averageEncoder())<Math.abs(drivingDistanceInEncoderTicks))
        //{
            robotAngle = Math.toRadians(getIMUAngle());
            LeftFront.setPower(speed * Math.cos(desiredAngle-robotAngle) + rightX);
            RightFront.setPower(speed * Math.sin(desiredAngle-robotAngle) - rightX);
            LeftBack.setPower(speed * Math.sin(desiredAngle-robotAngle) + rightX);
            RightBack.setPower(speed * Math.cos(desiredAngle-robotAngle) - rightX);
            outputTelemetry();
       // }
        stopRobot();

    }
    void DriveFieldRealtiveDistance(double power, double feet)
    {
        DriveFieldRealtive(power, getIMUAngle(), feet);
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
    void AlignWithDistance(double distanceCM, double power, double timeToAlign, double PIDangle, double PIDTurnignTime)
    {
        TurnPID(PIDangle, PIDTurnignTime);
        //AlignWithDistance(distanceCM, power, timeToAlign);
    }
    void AlignWithDistance(double distanceCM, double power, double timeToAlign)
    {
        // runtime.reset();
        // while (runtime.seconds() < timeToAlign)
        // {
        //     while (distanceCM>rangeSensor.getDistance(DistanceUnit.CM)) DriveLeft(power);
        //     while (distanceCM<rangeSensor.getDistance(DistanceUnit.CM)) DriveLeft(power*-1);
        // }
        // stopRobot();
    }
    void driveIntoMineral(double angleParrelToMinerals, int mineralLocation)
    {
        if (tfod!=null)
        {
            tfod.shutdown();
        }
        switchXandY();
        if (goldMineralLocation!=GOLD_MINERAL_CENTER) TurnPID(0, 1);
        if (mineralLocation==GOLD_MINERAL_RIGHT)
        {
            //setEquation(new double[] {38}); //47
            //driveOverLinePID(20, 5, 3);
            //DriveToPointPID(30,38, 3);
            DriveToPointPID(18,38, 1.5);
            //telemetry.log().add("First movement done");
            //strafeToDistanceXPID(30, 2);
            //telemetry.log().add("Second movement done");
            //setEquation(new double[] {3});
            //driveOverLinePID(23, 5, 3);
            DriveToPointPID(20,5, 1.2);
            //telemetry.log().add("Third movement done");
            ///strafeToDistanceXPID(5, 3);
            //telemetry.log().add("Forth movement done");
            // driveOverLine(.5, 10, 72, false);
            // // driveOverLine(.4, 50, 50, true);
        }
        else if (mineralLocation==GOLD_MINERAL_LEFT)
        {
            //setEquation(new double[] {15}); //25
            //driveOverLinePID(30, 5, 3);
            DriveToPointPID(50,13, 1.5);
            DriveToPointPID(34,11, .8);
            //setEquation(new double[] {3});
            // driveOverLinePID(24, 5, 3);
            DriveToPointPID(24,5, 1);
        }
        else if (goldMineralLocation==GOLD_MINERAL_CENTER)
        {
            DriveforLength(3.4, -.7);
        }
        // setEquation(new double[] {-1,80});
        // driveOverLine(.5, 60, 60, false);
        telemetry.log().add("third movement done");
    }
    double getAngledPositionX()
    {
        double currentPos=getRobotPositionX();
        double currentAngle=make90(getIMUAngle());
        if (currentAngle==0) return currentPos;  //tan will not work if 0 is the angle, and this method is useless without a angle
        return (currentPos*Math.cos(Math.toRadians(currentAngle)));
    }
    double getAngledPositionY()
    {
        double currentPos=getRobotPositionY();
        double currentAngle=make90(getIMUAngle());
        if (currentAngle==0) return currentPos;  //tan will not work if 0 is the angle, and this method is useless without a angle
        return (currentPos*Math.cos(Math.toRadians(currentAngle)));
    }
    double make90(double angle)
    {
        while (angle>=90) angle-=90;
        while (angle<0) angle+=90;
        return angle;
    }
    void driveIntoMineralCreator(double angleParrelToMinerals, int mineralLocation)
    {
        if (tfod!=null)
        {
            tfod.shutdown();
        }
        if (goldMineralLocation!=GOLD_MINERAL_CENTER) TurnPID(0, 1);
        if (goldMineralLocation==GOLD_MINERAL_RIGHT)
        {
            //strafeToDistanceXPID(39,3);
            //strafeToDistanceYPID(24,3);
             DriveToPointPID(24,40,2);
             DriveforLength(1,.5);
             //DriveToPointPID(35,40,2);
             DriveToPointPID(63,3,2);
            //setEquation(new double[] {41}); //47
           // driveOverLinePID(24,5,3);
            //telemetry.log().add("First movement done");
            ///strafeToDistanceYPID(36,3);
            //driveOverLine(.5, 5, 28, false);
            //setEquation(new double[] {-1,75});
            //driveOverLinePID(50, 5, 3);
            //strafeToDistanceXPID(3,3);
            //telemetry.log().add("third movement done");
        }
        else if (goldMineralLocation==GOLD_MINERAL_LEFT)
        {
            //setEquation(new double[] {20}); //25
            //driveOverLinePID(44, 5,3);
            DriveToPointPID(46,20, 1.5);
            DriveToPointPID(41,20, 1);
            DriveToPointPID(49,15, .5);
            //setEquation(new double[] {20}); //25
            DriveToPointPID(63,3,1.2);
            //driveOverLinePID(49, 5,3);
            //strafeToDistanceXPID(3, 3);
            // setEquation(new double[] {3}); //25
            // driveOverLine(.5, 37,55, false);
        }
        else if (goldMineralLocation==GOLD_MINERAL_CENTER)
        {
            DriveforLength(1, -.5);
            DriveforLength(1, .5);
            TurnPID(0,3);
            DriveToPointPID(63,3,2);
            //setEquation(new double[] {30}); //36
            //driveOverLinePID(34,5,1.5);
            //DriveToPointPID(34,34,1.5);
            //DriveToPointPID(48,34,2);
            //telemetry.log().add("First movement done");
            //strafeToDistanceYPID(3, 49);
            //driveOverLine(.5, 30, 43, false);
            //setEquation(new double[] {-1,63});
            //driveOverLinePID(51, 5, 3);
             //DriveToPointPID(49,3,2);
            //strafeToDistanceXPID(3, 3);
            //telemetry.log().add("third movement done");
        }
        sleep(.5);
    }
    void strafeToDistanceX(double power, double inch)
    {
        if (getRobotPositionY()-inch>0) while (getRobotPositionY() > inch && opModeIsActive())
        {
            DriveFieldRealtiveSimple(power, 0);
            telemetry.addData("postionY", getRobotPositionY());
            telemetry.update();
        }
        else while (getRobotPositionY() < inch && opModeIsActive())
        {
            DriveFieldRealtiveSimple(power, 180);
            telemetry.addData("postionY", getRobotPositionY());
            telemetry.update();
        }
        stopRobot();
    }
    void strafeToDistanceY(double power, double inch)
    {
        if (getRobotPositionX()-inch>0) while (getRobotPositionX() > inch && opModeIsActive())
        {
            DriveFieldRealtiveSimple(power, 270);
            telemetry.addData("postionY", getRobotPositionX());
            telemetry.update();
        }
        else while (getRobotPositionX() < inch && opModeIsActive())
        {
            DriveFieldRealtiveSimple(power, 90);
            telemetry.addData("postionY", getRobotPositionX());
            telemetry.update();
        }
        stopRobot();
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
    // void driveIntoMineralCreator(double angleParrelToMinerals, int mineralLocation)
    // {
    //     double feetToDrive=3.6;
    //     //get to the mineral
    //     if (mineralLocation==GOLD_MINERAL_RIGHT)
    //     {
    //         TurnPID(angleParrelToMinerals, 3);
    //         DriveforLength(1, .5); //back up to align with right mienral
    //         feetToDrive+=1;
    //     }
    //     else if (mineralLocation==GOLD_MINERAL_LEFT)
    //     {
    //         TurnPID(angleParrelToMinerals, 3);
    //         DriveforLength(1, -.5); //drive to align with left mienral
    //         feetToDrive-=1.03;
    //     }
    //     else if (mineralLocation==GOLD_MINERAL_CENTER) feetToDrive+=0; //do nothing becuase the robot is aleadry next to the center mineral, code just here so it's easier to read
    //     TurnPID(angleParrelToMinerals-90, 3);
    //     DriveforLength(.5, -.8);
    //     DriveforLength(.6, .5);
    //     TurnPID(angleParrelToMinerals, 3);

    //     floatMotors();
    //     DriveforLength(feetToDrive, -.9);
    //     brakeMotors();
    // }
    public void switchXandY()
    {
        //ModernRoboticsI2cRangeSensor temp =  distanceSensorY;
        //distanceSensorY= distanceSensorX;
        //distanceSensorX=temp;
        if (rotationOffset==90) setOffset(0,0,0);
        else setOffset(0,0,90);
    }
    public void setEquation(double[] equation)
    {
        terms=equation;
    }
    String writeEquation()
    {
        String currentEquation="f(x)=";
        for (int i=0; i<terms.length-2; i++)
        {
            if (terms[i]>0 && i!=0) currentEquation+=" + ";
            else if (terms[i]<0) currentEquation+=" - ";
            currentEquation+=(Double.toString(terms[i]) + "x^" + (terms.length-1-i)) + " ";
        }
        try {
            if (terms[terms.length-2]>0) currentEquation+=" + ";
            else currentEquation+=" - ";
            currentEquation+=Double.toString(terms[terms.length-1]) + "x ";
        } catch (Exception e) {}
        if (terms[terms.length-1]>0) currentEquation+=" + ";
        else currentEquation+=" - ";
        currentEquation+=Double.toString(terms[terms.length-1]);
        return currentEquation;
    }
    /*public void driveOverLinePID(double x, double seconds, double ratio)
    {
        driveOverLinePID(terms, x, seconds, ratio);
    }
    public void driveOverLinePID(double[] equation, double x,double seconds, double ratio)
    {
        terms=equation;
        double currentXPos;
        double disiredAngle;

        MiniPID miniPID = new MiniPID(.055, 0.000, 0.04);
        double targetX = x;
        miniPID.setSetpoint(0);
        miniPID.setSetpoint(targetX);
        miniPID.setOutputLimits(1);

        miniPID.setSetpointRange(40);

        double actualX=0;
        double outputX=0;

        MiniPID miniPIDY = new MiniPID(.25, 0.00, 0.3);
        double targetY = calculateValue(x);
        miniPIDY.setSetpoint(0);
        miniPIDY.setSetpoint(targetY);
        miniPIDY.setOutputLimits(1);

        miniPIDY.setSetpointRange(40);

        double actualY=0;
        double outputY=0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<seconds) {
            actualX = getRobotPositionX();
            outputX = miniPID.getOutput(actualX, targetX);
            actualY = getRobotPositionY();
            outputY = miniPIDY.getOutput(actualY, targetY);
            double power = Math.max(Math.abs(outputX), Math.abs(outputY));
            disiredAngle=getDisiredAngle(actualX, outputX<=0, ratio);
            //if () disiredAngle=invertAngle(disiredAngle);
            DriveFieldRealtiveSimple(power, disiredAngle+90);

            telemetry.addData("ANGLE",  disiredAngle);
            //telemetry.addData("Power", power);
            telemetry.addData("X output", outputX);
            telemetry.addData("X pos", actualX);
            telemetry.addData("Y output", outputY);
            telemetry.addData("Y pos", actualY);
            telemetry.update();
        }
        stopRobot();
        telemetry.log().add("STOPPED");
    }
    */
   public void DriveToPointPID(double x, double y, double seconds)
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
            if (power>.8) power = .8;
            double slope = getSlope(x, y, actualX, actualY);
            double angle = Math.toDegrees(Math.atan2(outputX, -outputY));
            DriveFieldRealtiveSimple(power, angle);
            telemetry.addData("angle", angle);
            telemetry.addData("power",power);
            telemetry.addData("X pos", actualX);
            telemetry.addData("Y pos", actualY);
            telemetry.addData("outputX", outputX);
            telemetry.addData("outputY", outputY);
            telemetry.update();
            if (Math.abs(getIMUAngle())>10)
            {
                TurnPID(0,.5);
                seconds+=.5;
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
        if (Double.isNaN(distance))
        {
            telemetry.log().add("DISTANCE VALUE WAS NaN");
            return 2.5;
        }
        if (distance==0)
        {
            stopRobot();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
            distanceSensorX = hardwareMap.get(Rev2mDistanceSensor.class, "range sensor2");
            distance=(distanceSensorX.getDistance(DistanceUnit.INCH));
            telemetry.log().add("DISTANCE SENSOR X BROKE!!!!!!!");
            sleep(1d);
        }
        if (distance>1440)
        {
            telemetry.log().add("overshot X distance");
            return lastPosXrev;
        }
        lastPosXrev=distance;
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
        /*if (isRobotInX)
        {
            if (distanceBest>lastPosX+12)
            {
                telemetry.log().add("ROBOT LEFT ON X DISTANCE");
                isRobotInX=false;
                lastPosX=distanceBest;
                return distanceBest;
            }
            else
            {
                distanceBest+=XDifference;
            }
        }
        if (distanceBest<lastPosX-12 && !isRobotInX)
        {
            telemetry.log().add("ROBOT DETECTED ON X DISTANCE");
            isRobotInX=true;
            XDifference=lastPosX-distanceBest;
            lastPosX=distanceBest;
            distanceBest-=XDifference;
        }*/
        lastPosX=distanceBest;
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
            distance=lastPosY;
        }
        /*if (isRobotInY)
        {
            if (distance>lastPosY+12)
            {
                telemetry.log().add("ROBOT LEFT ON Y DISTANCE");
                isRobotInY=false;
                lastPosY=distance;
                return distance;
            }
            else
            {
                distance+=YDifference;
            }
        }
        else if (distance<lastPosY-12 && !isRobotInY)
        {
            telemetry.log().add("ROBOT DETECTED ON Y DISTANCE");
            isRobotInY=true;
            YDifference=lastPosY-distance;
            lastPosY=distance;
            distance-=YDifference;
        }*/
        lastPosY=distance;
        return distance;
    }
    public boolean isLimitSwitchPressed()
    {
        if (limitSwitch.getVoltage()>2.8) return true;
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
    public void stopRobot()
    {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }
    public void runOpMode() {}
}
