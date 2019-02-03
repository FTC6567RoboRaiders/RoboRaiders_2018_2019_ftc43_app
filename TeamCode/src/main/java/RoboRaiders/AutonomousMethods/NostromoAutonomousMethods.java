package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.NostromoBotMotorDumper;
import RoboRaiders.Robot.RobotTelemetryDisplay;


public abstract class NostromoAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this, "Nostormo");

    public void farRedDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID(robotPID, robot, 50);

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(50));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        //DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        encodersMove(robot, 6, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);


        imuTurn(robot, 60, .25, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        encodersMove(robot, 30, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(30));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        encodersMove(robot, 15, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(15));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(15));

    }

    public void closeRedDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        //DeployRobot(robot);

        EncoderDrivePID(robot, 28);

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(28));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        encodersMove(robot, 3, 1, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        imuTurn(robot, 100, .35, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To  Be Turned", String.valueOf(100));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(100);

        EncoderDrivePID(robot, 41);   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(36));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        robot.collectionOff();
        rtd.displayRobotTelemetry("Turning Off Collection");

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        // DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        imuTurn(robot, 90, .35, "right"); // was 85, now at 90

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(200);

        EncoderDrivePID(robot, 68);  // was 78 inches

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(250);
    }

    public void farBlueDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID(robotPID, robot, 28);
        robotSleep(500);

        encodersMove(robot, 6, .80, "backward");
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 35);
        robotSleep(500);

        imuTurn(robot, 40, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 34);
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");
        robotSleep(500);

        imuTurn(robot, 90, .30, "right");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 78);
        robotSleep(500);
    }

    public void closeBlueDepot(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID(robotPID, robot, 48);
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 1);
        robotSleep(500);

        imuTurn(robot, 45, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 60);
        robotSleep(500);
    }

    public void moveTest(RoboRaidersPID robotPID, NostromoBotMotorDumper robot) {
        EncoderDrivePID(robotPID, robot, 48);
    }

    public void moveDepotFromCraterStart(NostromoBotMotorDumper robot) {


        EncoderDrivePID(robot, 35);   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Moving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 65, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward");
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 30, .35, "right");


    }

    public void moveDepotFromDepotStart(NostromoBotMotorDumper robot) {
        encodersMove(robot, 6,0.8,"Forward");//also mineral knocking was 50

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

    }

    public void parkFromCraterStart(NostromoBotMotorDumper robot) {

        encodersMove(robot, 68, 0.8, "Forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(250);
    }

    public void parkFromDepotStart(NostromoBotMotorDumper robot) {

       encodersMove(robot, 54.5,.5,"backward");
       robotSleep(500);
    }


    public void EncoderDrivePID(RoboRaidersPID robotPID, NostromoBotMotorDumper robot, double wantedDistance) {
        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about

        double EncoderCount = robot.calculateCOUNTS(wantedDistance);
        double currentEncoderCount = robot.getSortedEncoderCount();
        while (opModeIsActive() &&
                (currentEncoderCount <= EncoderCount - 30.0 || currentEncoderCount >= EncoderCount + 30.0)) {

            motor_power = robotPID.pidWithCounts(EncoderCount, robot.getSortedEncoderCount());
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);
            currentEncoderCount = robot.getSortedEncoderCount();


        }
    }

    /* public void imuTurnWithPID (NostromoBot robot, float degrees, String direction ) {
         robot.resetIMU();
         float finalHeading = robot.getHeading() + degrees;

         // robot.getHeading(); returns the current heading of the IMU

         if (direction.equals("right")) { //if the desired direction is right

             robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
         }
         else if (direction.equals("left")) { //if the desired direction is left

             robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
         }

         while (robot.getHeading() < (finalHeading - 20) && opModeIsActive()) { //while the value of getHeading is
             //less then the degree value
             //and while opMode is active continue the while loop

             telemetry.addData("Heading", robot.getHeading()); //feedback of getHeading value
             telemetry.update(); //continuous update
         }

         robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
     }
     }*/
    public void EncoderDrivePID(NostromoBotMotorDumper robot, double wantedDistance) {
        //    robot.resetEncoders();
        //    robot.runWithEncoders();
        RoboRaidersPID pidClass = new RoboRaidersPID();   // create new pidClass

        EncoderDrivePID(pidClass, robot, wantedDistance);

    }

    public void imuTurn(NostromoBotMotorDumper robot, float degreesToTurn, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = robot.getIntegratedZAxis();
        //finalHeading = currentHeading + degreesToTurn;
        //telemetry.update();

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() > finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();

            }
        }
        else { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;
            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
            while(opModeIsActive() && robot.getIntegratedZAxis() < finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();
            }
        }


        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }


    public void DeployRobot(NostromoBotMotorDumper robot) {

        double startDeployTime = System.currentTimeMillis();

        robot.setLiftMotorPower(-0.95);
        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime < 1500 && !robot.sensorTouch.isPressed()) {

        }

        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime >= 1500 && System.currentTimeMillis() - startDeployTime < 5000 && !robot.sensorTouch.isPressed()) {
            robot.setLiftMotorPower(-.45);
        }


        //while (opModeIsActive() && !robot.sensorTouch.isPressed() && System.currentTimeMillis()-startDeployTime < 7800) {
        // System.currentTimeMillis()-startDeployTime is the elapsed time (the current time minus the start time)
        //}

        robot.setLiftMotorPower(0);

        //robotSleep (250);

        //encodersMove(robot, 1.5, .5, "backward");

        robotSleep(500);

        robot.liftClaw.setPosition(robot.liftClawOpen);

        robotSleep(1200);

    }

    public void DeployTeamMarker(NostromoBotMotorDumper robot) {
       /* long t = System.currentTimeMillis();
        long end = t + 500;
        while (System.currentTimeMillis() < end) {//up
            robot.dumpp1.setPosition(robot.dumpdirection1);
            robot.dumpp2.setPosition(robot.dumpdirection2);
            robotSleep(500);
        }
        robot.dumpp1.setPosition(robot.dumpdirectionstop);
        robot.dumpp2.setPosition(robot.dumpdirectionstop);

        robot.dumpWrist.setPosition(robot.dropTeamMarker);//put elbow down
        robotSleep(500);

        while (System.currentTimeMillis() < end) {//down
            robot.dumpp1.setPosition(robot.dumpdirection1);
            robot.dumpp2.setPosition(robot.dumpdirection2);
            robotSleep(500);
        }
        robot.dumpp1.setPosition(robot.dumpdirectionstop);
        robot.dumpp2.setPosition(robot.dumpdirectionstop);

        while (System.currentTimeMillis() < end){ //up
            robot.dumpp1.setPosition(robot.dumpdirection1);
            robot.dumpp2.setPosition(robot.dumpdirection2);
            robotSleep(500);
        }
        robot.dumpp1.setPosition(robot.dumpdirectionstop);
        robot.dumpp2.setPosition(robot.dumpdirectionstop);

        robot.dumpWrist.setPosition(robot.bringMarkerBack);
        robotSleep(500);*/

      /* robot.dumperUp();
       robotSleep(750);
       robot.dumperStop();

       robot.dumpWrist.setPosition(robot.dumpTeamMarkerWristDump);
       robotSleep(500);

       robot.dumpWrist.setPosition(robot.dumpWristNotDump);
       robot.dumperDown();
       robotSleep(1000);
       robot.dumperStop();*/


        }
    /**
     * Will detect the location of the gold mineral
     *
     * @param robot - the robot to work with
     */
    public void samplingMineralsDepot(NostromoBotMotorDumper robot) {

        encodersMove(robot, 4, .6, "forward");
        robotSleep(200);

        imuTurn(robot, 90, .45,"left");
        robotSleep(200);

        int goldLocation = detectGoldMineral(robot);

        telemetry.addLine().addData("Mineral Seen", String.valueOf(goldLocation));
        telemetry.update();

        switch (goldLocation) {

            case 1:
                mineralLeftDepot(robot);
                break;
            case 2:
            case -1:
                mineralCenterDepot(robot);
                break;
            case 3:
                mineralRightDepot(robot);
                break;

        }
    }
    /**
     * Will detect the location of the gold mineral
     *
     * @param robot - the robot to work with
     */
    public void samplingMineralsCrater(NostromoBotMotorDumper robot) {

        encodersMove(robot, 4, .6, "forward");
        robotSleep(200);

        imuTurn(robot, 90, .45,"left");
        robotSleep(200);

        int goldLocation = detectGoldMineral(robot);

        switch (goldLocation) {
            case 1:
                mineralLeftCrater(robot);
                break;
            case 2:
            case -1:
                mineralCenterCrater(robot);
                break;
            case 3:
                mineralRightCrater(robot);
                break;
        }
    }

    /**
     * will detect the gold mineral location
     *
     * @return the location of the gold mineral
     */
    public int detectGoldMineral(NostromoBotMotorDumper robot) {
        int goldPostion = -1;
        int numberofrecognized = 0;
        List<Recognition> updatedRecognitions = null;
        double startSamplingTime = System.currentTimeMillis();

        CameraDevice.getInstance().setFlashTorchMode(true);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */

            //took out turning on the flash for the second time
            //took out a 1.5 second wait
            while (opModeIsActive() && System.currentTimeMillis() - startSamplingTime <= 1500 && numberofrecognized <= 1) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions == null) {
                    numberofrecognized = 2;
                } else {
                    numberofrecognized = updatedRecognitions.size();
                }

            }//while?

            if (updatedRecognitions == null) {
            } else {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // Is the robot seeing at least one mineral  THIS IS CHANGED, WE NOW JUST CARE IF WE SEE AT LEAST ONE THING
                if (updatedRecognitions.size() >= 1) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    telemetry.addData("goldMineralX", String.valueOf(goldMineralX));
                    telemetry.addData("silverMineral1X", String.valueOf(silverMineral1X));
                    telemetry.addData("silverMineral2X", String.valueOf(silverMineral2X));


                    // Did the robot not see the gold mineral    THIS IS CHANGED
                    if (goldMineralX == -1) {

                        // Yes, indicate the gold mineral is on the left
                        goldPostion = 1;
                    }
                    // The robot saw a gold mineral find where it is
                    else {

                        // Is the gold mineral to the left of the silver mineral?  THIS IS CHANGED
                        if (goldMineralX < 500) {

                            // Yes, indicate the gold mineral is in the center
                            goldPostion = 2;
                        } else {

                            // No, the gold mineral is on the right
                            goldPostion = 3;
                        }
                    }
                }
                telemetry.update();
            }

        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
        return goldPostion;

    }



    public void mineralLeftDepot(NostromoBotMotorDumper robot) {

        encodersMove(robot, 6, .8, "forward");
        robotSleep(500);

        imuTurn(robot, 90, .45, "right");
        robotSleep(500);

        encodersMove(robot, 15.5, .8, "forward");
        robotSleep(500);

        imuTurn(robot, 50, .45, "left");
        robotSleep(500);

        encodersMove(robot, 3, .4, "forward");
        robotSleep(500);

        imuTurn(robot, 90, .45, "right");
        robotSleep(500);
    }

    public void mineralRightDepot(NostromoBotMotorDumper robot) {

        encodersMove(robot, 6, .8, "backward");
        robotSleep(500);

        imuTurn(robot, 90, .45, "right");
        robotSleep(500);

        encodersMove(robot, 15, .8, "forward");
        robotSleep(500);

        imuTurn(robot, 95, .45, "left");
        robotSleep(500);

        encodersMove(robot, 28, .8, "forward");
        robotSleep(500);

        imuTurn(robot, 110, .45, "right");
        robotSleep(500);

    }

    public void mineralCenterDepot(NostromoBotMotorDumper robot)  {

        imuTurn(robot,90,.45,"right");

        encodersMove(robot, 18, .8, "forward");
        robotSleep(200);

        imuTurn(robot,90,.45,"left");
        robotSleep(200);

        encodersMove(robot,15,.5,"forward");
        robotSleep(200);

        imuTurn(robot, 110,.45,"right");
        robotSleep(200);


    }


    /////crater thing now

    public void  mineralLeftCrater(NostromoBotMotorDumper robot) {

        encodersMove(robot, 8, .5, "forward");
        robotSleep(500);

        imuTurn(robot, 90, .45, "right");
        robotSleep(500);

        encodersMove(robot, 7, .5, "forward");
        robotSleep(500);

        encodersMove(robot, 5, .5, "backward");
        robotSleep(500);

        imuTurn(robot, 100, .45, "left");
        robotSleep(500);

        encodersMove(robot, 28,.5,"forward");
        robotSleep(500);

        imuTurn(robot, 45, .45, "left");
        robotSleep(500);

        encodersMove(robot,42,.5,"forward");
        robotSleep(500);
    }

    public void mineralRightCrater(NostromoBotMotorDumper robot) {

        encodersMove(robot, 8, .5, "backward");
        robotSleep(500);

        imuTurn(robot, 90, .45, "right");
        robotSleep(500);

        encodersMove(robot, 7, .5, "forward");
        robotSleep(500);

        encodersMove(robot, 5, .5, "backward");
        robotSleep(500);

        imuTurn(robot, 100, .45, "left");
        robotSleep(500);

        encodersMove(robot, 62, .5, "forward");
        robotSleep(500);

        imuTurn(robot, 45, .45,"left");
        robotSleep(500);

        encodersMove(robot,42,.5,"forward");
        robotSleep(500);

    }

    public void mineralCenterCrater(NostromoBotMotorDumper robot)  {

        imuTurn(robot, 90,.45,"right");

        encodersMove(robot, 6, .8, "forward");
        robotSleep(500);

        encodersMove(robot, 3, 1.0, "backward");
        robotSleep(500);

        imuTurn(robot, 95, .45, "left");
        robotSleep(500);

        encodersMove(robot,46,.5, "forward");
        robotSleep(500);

        imuTurn(robot, 45, .45,"left");
        robotSleep(500);

        encodersMove(robot,42,.5,"forward");
        robotSleep(500);
    }







    /*  public void DistanceDrivePID() {
        while (opModeIsActive() && robot.getSensorDistance() < Target) {
            motor_power = drivePID.pidWithDistance(robot.getSensorDistance(), Target);
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

            telemetry.addData("Distance Sensor", robot.getSortedEncoderCount());
            telemetry.addData("Target Distance", Target);
            telemetry.update();

        }*/
    public void encodersMove(NostromoBotMotorDumper robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.calculateCOUNTS(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            robot.setDriveMotorPower(power, power, power, power); //start driving forward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("left")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, power, -power); //start strafing left

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }

        robot.runWithoutEncoders(); //sets the mode back to run without encoder
    }



    /**
     * make the robot sleep (wait)
     *
     * @param timeToSleep time in milliseconds
     */
    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
