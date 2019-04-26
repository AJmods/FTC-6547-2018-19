/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link revIMUVelocityTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Rev Veliocty test", group = "Sensor")
// Comment this out to add to the opmode list
public class REVIMUTest extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Velocity velocity;
    Position position;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    boolean feildRealtive=true;

    boolean isGamepadPressed=false;

    double speedModifer=.6;
    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

       // INIT(hardwareMap);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //NaiveAccelerationIntegrator naiveAccelerationIntegrator = new NaiveAccelerationIntegrator();
        parameters.accelerationIntegrationAlgorithm = new CobaltColtAccerlerationIntgrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.log().add("ready to start");
        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(DistanceUnit.INCH,0,0,0,0), new Velocity(DistanceUnit.INCH,0,0,0,0), 1000);

        double velocityX=0;
        double velocityY=0;
        double velocityZ=0;
        double oldTime=System.currentTimeMillis();
        double newTime=System.currentTimeMillis();
        // Loop and update the dashboard
        while (opModeIsActive()) {
            newTime=System.currentTimeMillis();
            double time = (newTime-oldTime)/1000;
            velocityX+=gravity.xAccel*time;
            velocityY+=gravity.yAccel*time;
            velocityZ+=gravity.xAccel*time;
            // if (feildRealtive)
            // {
            //     //if (gameTime.seconds()<90) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
            //     double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //     double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //     //if (offset) LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4)+Math.toRadians(45);
            //     double robotAngle = Math.toRadians(getIMUAngle());
            //     double rightX=gamepad1.right_stick_x;
            //     leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
            //     rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
            //     leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
            //     rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;

            //     if (gamepad1.x && !isGamepadPressed)
            //     {
            //         feildRealtive = false;
            //         isGamepadPressed=true;
            //     } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            // }
            // else
            // {
            //     //if (gameTime.seconds()<90) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            //     leftFrontPower=-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            //     rightFrontPower=-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            //     leftBackPower=-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            //     rightBackPower=-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

            //     if (gamepad1.x && !isGamepadPressed)
            //     {
            //         feildRealtive = true;
            //         isGamepadPressed=true;
            //     } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            // }
            // LeftFront.setPower(leftFrontPower*speedModifer);
            // RightFront.setPower(rightFrontPower*speedModifer);
            // LeftBack.setPower(leftBackPower*speedModifer);
            // RightBack.setPower(rightBackPower*speedModifer);

            telemetry.addData("Velocity X", velocityX);
            telemetry.addData("Velocity Y", velocityX);
            telemetry.addData("Velocity Z", velocityX);
            telemetry.addData("Position", position.toString());
            telemetry.update();
            oldTime=System.currentTimeMillis();
        }
    }
    double getRobotPositionXIMU() {return position.x;}
    double getRobotPositionYIMU() {return position.y;}
    double getRobotPositionZIMU() {return position.z;}
    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getAcceleration();
                velocity = imu.getVelocity();
                position = imu.getPosition();
                }
            });

        telemetry.addLine()
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
                });
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
}
