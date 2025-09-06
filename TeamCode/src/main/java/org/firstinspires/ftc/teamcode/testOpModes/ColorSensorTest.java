package org.firstinspires.ftc.teamcode.testOpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ColorSensorTest", group = "Autonomous")
public class ColorSensorTest extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map (name: "colorSensor")
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Read normalized RGBA values
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert normalized floats (0..1) to 0..255 ints for Android Color conversion
            int r = (int) (colors.red * 255);
            int g = (int) (colors.green * 255);
            int b = (int) (colors.blue * 255);

            // Convert RGB to HSV to get hue in degrees (0..360)
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);
            float hue = hsv[0];

            // Determine if color is green or purple using hue ranges
            // These ranges are adjustable based on your sensor and lighting
            boolean isGreen = (hue >= 145 && hue <= 185);    // typical green range
            boolean isPurple = (hue >= 200 && hue <= 240);  // typical purple range

            // Telemetry output based on detection
            if (isGreen) {
                telemetry.addData("Detected Color", "green");
            } else if (isPurple) {
                telemetry.addData("Detected Color", "purple");
            } else {
                telemetry.addData("Detected Color", "not green and not purple");
            }

            // Helpful debug values
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("R G B", "%d %d %d", r, g, b);
            telemetry.update();

            // Small delay to avoid flooding telemetry
            sleep(100);
        }
    }
}
