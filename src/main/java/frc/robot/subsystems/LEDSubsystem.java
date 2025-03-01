// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Controls.HubturmState;
import frc.robot.Controls.IntakeState;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED ledsRight;
    private AddressableLED ledsLeft;

    private AddressableLEDBuffer ledsBuffer;

    private static class RGB {
        public int red;
        public int green;
        public int blue;

        public RGB(int gray) {
            red = gray;
            green = gray;
            blue = gray;
        }

        public RGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static RGB coral = new RGB(0, 0, 255);
    private static RGB climb = new RGB(255, 0, 0);
    private static RGB noLight = new RGB(0);

    public LEDSubsystem() {
        ledsLeft = new AddressableLED(Constants.LEDs.ledPortLeft);
        ledsRight = new AddressableLED(Constants.LEDs.ledPortRight);
        ledsRight.setLength(Constants.LEDs.ledBufferLength);

        ledsBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);

        ledsRight.start();
        ledsLeft.start();

        normalLeds();
        setData();
    }

    private void setData() {
        ledsRight.setData(ledsBuffer);
        ledsLeft.setData(ledsBuffer);
    }

    @Override
    public void periodic() {
    }

    private void setLEDsInBlocks(int blockLen, int blockOffsetFromStart, RGB color1, int lenCol1, RGB color2) {
        for (int i = blockOffsetFromStart; i < Constants.LEDs.ledBufferLength; i += blockLen) {
            for (int j = 0; j < blockLen; j++) {
                // Für die ersten drei LEDs die Farbe von "algue", danach aus.
                if (j < lenCol1) {
                    ledsBuffer.setRGB(i + j, color1.red, color1.green, color1.blue);
                } else if (j < blockLen) {
                    ledsBuffer.setRGB(i + j, color2.red, color2.green, color2.blue);
                }
            }
        }
        setData();
    }

    private void setLEDsVertical(RGB ledColorBottom, RGB ledColorTop){
        ledsBuffer.setRGB(0, ledColorBottom.red, ledColorBottom.green, ledColorBottom.blue);
        ledsBuffer.setRGB(1, ledColorTop.red, ledColorTop.green, ledColorTop.blue);
        ledsBuffer.setRGB(2, ledColorBottom.red, ledColorBottom.green, ledColorBottom.blue);
        ledsBuffer.setRGB(3, ledColorTop.red, ledColorTop.green, ledColorTop.blue);
    }

    private void setAllLEDs(RGB color) {
        for (int i = 0; i < Constants.LEDs.ledBufferLength; i++) {
            ledsBuffer.setRGB(i, color.red, color.green, color.blue);
        }
        setData();
    }

    public void coralIntakeLEDs() {
        setLEDsInBlocks(4, 0, coral, 1, noLight);
        setData();
    }

    public void coralL2OuttakeLEDs() {
        setLEDsInBlocks(Constants.LEDs.ledBufferLength, 0, coral, 2 * Constants.LEDs.ledBufferLength / 4, noLight);
        setData();
    }

    public void coralL3OuttakeLEDs() {
        setLEDsInBlocks(Constants.LEDs.ledBufferLength, 0, coral, 3 * Constants.LEDs.ledBufferLength / 4, noLight);
        setData();
    }

    public void coralL4OuttakeLEDs() {
        setAllLEDs(coral);
        setData();
    }

    
    public void rainbowAnimationLEDs() {
        LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);
        Distance kLedSpacing = Distance.ofBaseUnits(1 / 120, Meters);
        LEDPattern m_scrollingRainbow = rainbowPattern
        .scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(1, MetersPerSecond), kLedSpacing);
        m_scrollingRainbow.applyTo(ledsBuffer);
        setData();
    }
    
    public void climbLEDs() {
        rainbowAnimationLEDs();
    }
    public void normalLeds() {
        setAllLEDs(climb);
    }

    //TODO: Implement this method
    public void synchronizeLEDsWithStates() {
        HubturmState hubturmState = RobotContainer.controls.getActiveLiftingTowerState();
        IntakeState intakeState = RobotContainer.controls.activeIntakeState;

        

        if (intakeState == IntakeState.INTAKE) {
            coralIntakeLEDs();
        } else {
            switch (hubturmState) {
                case LTWO:
                    coralL2OuttakeLEDs();
                    break;
                case LTHREE:
                    coralL3OuttakeLEDs();
                    break;
                case LFOUR:
                    coralL4OuttakeLEDs();
                    break;
                default:
                    break;
            }
        }
    }
}