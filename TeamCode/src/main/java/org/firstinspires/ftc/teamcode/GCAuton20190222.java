package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Drew from 11874 on 10/20/2018.
 */

@Autonomous(name = "GC Test 2019")
public class GCAuton20190222 extends theColt {

    ElapsedTime gametime = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addData("You broke it Harrison","");
        telemetry.update();

        INIT(hardwareMap);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

        initIMU();

        initTfod();

        zeroEncoders();

        angleZzeroValue=45;

        tfod.activate();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        while (!isStarted()) //scan until the program starts
        {
            scanMinerals();
        }
        gametime.reset();
        //if the program was started and the gold mineral wasn't detected, default to the GOLD_MINERAL_LEFT position
        if (goldMineralLocation==GOLD_MINERAL_UNKNOWN)
        {
            goldMineralLocation=GOLD_MINERAL_LEFT;
            telemetry.log().add("gold mineral UNKNOWN, defaulting to LEFT");
        }
        else telemetry.log().add("gold Mineral location "+ goldMineralLocation);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //Start IMU

        outputTelemetry();
        lowerRobot();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        sleep(.5);
        DriveforLength(1.23, -.2); //drive forward out of the lander
        sleep(.5);
        driveIntoMineral(-45, goldMineralLocation); //this method knocks over the gold mineral and goes to depot
        TurnPID(0,2); //make sure the robot is facing straight towards the depot for deploying the team marker
        if (goldMineralLocation==GOLD_MINERAL_LEFT && getRobotPositionX()>24) do
        {
            setEquation(new double[] {3});
            driveOverLine(.5, 24, 72, true);
            TurnPID(0,2);
        } while (getRobotPositionX()>24);
        if (goldMineralLocation==GOLD_MINERAL_RIGHT) DriveforLength(.8, -.5);
        deployTeamMarker();
        setEquation(new double[] {3});
        do {
            driveOverLinePID(38, 5, 3); //to halfway creater
            TurnPID(0, 2);
        } while (getRobotPositionX()<40 && (distanceSensorX.getDistance(DistanceUnit.INCH))!=0 && (distanceSensorY.getDistance(DistanceUnit.INCH))!=0 && gametime.seconds()<24);
        // XXX The above loop should use getRobotPositionX() and getRobotPositionY() instead of accessing the sensors directly.

        DriveFieldRealtiveDistance(.5, 90, 3.5);
        TurnPID(0,1);
        DriveFieldRealtiveDistance(.14, 90,.1);

        stopRobot();                                                    //at long distances, distance sensors are unreliable.
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle()); //save the current gyro angle for later use in the field realtive teleop
        teamMarker.setPosition(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        raiseRobot();
        telemetry.log().add("Autonomous is done, if might have failed but hopefully it didn't");
        while (opModeIsActive()) //after program is done
        {
            telemetry.addData("current angle", getIMUAngle());
            telemetry.update();
        }


    }
}
