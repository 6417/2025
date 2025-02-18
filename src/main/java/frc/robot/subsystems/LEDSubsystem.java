// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import javax.naming.Binding;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Controls.GamePieceState;
import frc.robot.Controls.HubturmState;
import frc.robot.Controls.IntakeState;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;

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

    private static RGB algue = new RGB(0, 175, 140);
    private static RGB coral = new RGB(255, 242, 199);
    private static RGB climb = new RGB(255, 0, 0);
    private static RGB noLight = new RGB(0);

    public LEDSubsystem() {
        ledsLeft = new AddressableLED(Constants.LEDs.ledPortLeft);
        ledsRight = new AddressableLED(Constants.LEDs.ledPortRight);
        ledsRight.setLength(Constants.LEDs.ledBufferLength);

        ledsBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);

        ledsRight.start();

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

    }

    private void setAllLEDs(RGB color) {
        for (int i = 0; i < Constants.LEDs.ledBufferLength; i++) {
            ledsBuffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void algueIntakeLEDs() {
        setLEDsInBlocks(3, 0, algue, 3, noLight);
        setData();
    }

    public void algueOuttakeLEDs() {
        setAllLEDs(algue);
        setData();
    }

    public void coralIntakeLEDs() {
        setLEDsInBlocks(3, 0, coral, 3, noLight);
        setData();
    }

    public void coralL1OuttakeLEDs() {
        setLEDsInBlocks(Constants.LEDs.ledBufferLength, 0, coral, Constants.LEDs.ledBufferLength / 4, noLight);
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

    public void climbLEDs() {
        setAllLEDs(climb);
        setData();
    }

    public void rainbowAnimationLEDs() {
        LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);
        Distance kLedSpacing = Distance.ofBaseUnits(1 / 120, Meters);
        LEDPattern m_scrollingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(1, MetersPerSecond), kLedSpacing);
    }

    public void normalLeds() {
        setAllLEDs(climb);
        setData();
    }

    public void synchronizeLEDsWithStates() {
        HubturmState hubturmState = RobotContainer.controls.getActiveLiftingTowerState();
        IntakeState intakeState = RobotContainer.controls.getActiveIntakeState();
        GamePieceState gamePieceState = RobotContainer.controls.getActivePieceState();
        switch (gamePieceState) {
            case ALGUE:
                if (intakeState == IntakeState.INTAKE) {
                    algueIntakeLEDs();
                } else {
                    algueOuttakeLEDs();
                }
                break;
            case CORAL:
                if (intakeState == IntakeState.INTAKE) {
                    coralIntakeLEDs();
                } else {
                    switch (hubturmState) {
                        case LONE:
                            coralL1OuttakeLEDs();
                            break;
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
                break;
            default:
                break;
        }
    }

}