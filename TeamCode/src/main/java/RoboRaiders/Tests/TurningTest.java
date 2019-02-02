package RoboRaiders.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="TurningTest", group="Samples")


public class TurningTest extends LinearOpMode {
    public float degreesToTurn;
    public float currentHeading;
    public float finalHeading;
    Orientation angles;
    BNO055IMU imu;




    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.update();

        waitForStart();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        degreesToTurn = 20;
        telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = angles.firstAngle;
        finalHeading = currentHeading + degreesToTurn;
        telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));


        while(opModeIsActive() && currentHeading < finalHeading) {
        }

    }
}
