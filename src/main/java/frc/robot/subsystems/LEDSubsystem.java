package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDSubsystem extends SubsystemBase{

    private final CANdle m_candle = new CANdle(LEDConstants.CANdleID);
    private final int ledCount = LEDConstants.LEDcount;

    private boolean mutliplePatterns = false;
    private boolean animationOn = false;
    private double brightness;
    private double ledAnimSpeed;
    private int ledOffset;
    private int ledPocket = 1;
    private BounceMode bounce = BounceMode.Center;
    private boolean ledReversed;
    private Color ledColor = new Color(0, 0, 0);
    private int ledWhite;

    private int tempVarDeleteLaterPlease = 0;

    public enum animations {
        SET_ALL,
        FIRE_ANIM,
        LARSON_ANIM,
        RAINBOW_ANIM,
        RGB_FADE_ANIM, 
        SINGLE_FADE_ANIM,
        STROBE_ANIM,
        COLOR_FLOW_ANIM,
        TWINKLE_OFF_ANIM,
        TWINKLE_ANIM
    }

    public LEDSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        configAll.v5Enabled = true;
        m_candle.configAllSettings(configAll, 100);

        clear();
        // setColor(new Color(255, 0, 0));
        // setBrightness(1);
        // setAnimSpeed(0.25);
        // larsonAnimation();

        // setLEDs(0, 0, 100);
        // m_candle.animate(new RainbowAnimation(1, 0.5, 16, false, 0));
    }

    public int getR(Color color) {
        return (int)(color.red*255);
    }

    public int getG(Color color) {
        return (int)(color.green*255);
    }

    public int getB(Color color) {
        return (int)(color.blue*255);
    }

    /**
     * Sets the color of the LEDs
     */
    public LEDSubsystem setColor(Color color) {
        ledColor = color;
        mutliplePatterns = false;
        return this;
    }

    public LEDSubsystem setMultipleColors(Color color, int startIndx, int ledCount) {
        m_candle.setLEDs(getR(color), getG(color), getB(color), ledWhite, startIndx, this.ledCount);
        mutliplePatterns = true;
        return this;
    }

    /**
     * Sets the white Offset if applicable for the animation
     *
     * @param white - [0, 255]
     */
    public LEDSubsystem setWhite(int white) {
        ledWhite = white;
        return this;
    }

    public LEDSubsystem setBrightness(double brightness) {
        this.brightness = brightness;
        return this;
    }

    public LEDSubsystem setAnimSpeed(double animSpeed) {
        ledAnimSpeed = animSpeed;
        return this;
    }

    public LEDSubsystem setOffset(int offset) {
        ledOffset = offset;
        return this;
    }

    public LEDSubsystem setPocket(int pocket) {
        ledPocket = pocket;
        return this;
    }

    public LEDSubsystem setBounce(BounceMode bounce) {
        this.bounce = bounce;
        return this;
    }

    public void disableAnimation() {
        clear();
        animationOn = false;
    }

    public void fireAnimation() {
        m_candle.animate(new FireAnimation(brightness, ledAnimSpeed, ledCount, 1, 1, ledReversed, ledOffset), 0);
    }

    public void larsonAnimation() {
        m_candle.animate(new LarsonAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, bounce, ledPocket), 0);
    }
    
    public void rainbowAnimation() {
        m_candle.animate(new RainbowAnimation(brightness, ledAnimSpeed, ledCount, ledReversed, ledOffset), 0);
    }

    public void rgbFadeAnimation() {
        m_candle.animate(new RgbFadeAnimation(brightness, ledAnimSpeed, ledCount, ledOffset), 0);
    }

    public void singleFadeAnimation() {
        m_candle.animate(new SingleFadeAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, ledOffset), 0);
    }

    public void strobeAnimation() {
        m_candle.animate(new StrobeAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, ledOffset), 0);
    }

    public void colorFlowAnimation() {
        m_candle.animate(new ColorFlowAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset), 0);
    }

    public void twinkleOffAnimation() {
        m_candle.animate(new TwinkleOffAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset), 0);
    }

    public void twinkleAnimation() {
        m_candle.animate(new TwinkleAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset), 0);
    }

    public void teleoperation() {
        clear();
        setBrightness(1);
        setAnimSpeed(.5);
        setOffset(0);
        rainbowAnimation();
        // setBrightness(1);
        // setAnimSpeed(.5);
        // setColor(new Color(0, 150, 255));
        // setAnimation(animations.SINGLE_FADE_ANIM);
    }

    public void autonomous() {
        clear();
        setBrightness(1);
        setAnimSpeed(.5);
        setOffset(0);
        larsonAnimation();
    }

    public void disabled() {
        clear();
        // setBrightness(1);
        // setAnimSpeed(.5);
        // setOffset(0);
        // setAnimation(animations.RAINBOW_ANIM);
        setBrightness(1);
        setAnimSpeed(.5);
        setColor(new Color(255, 30, 0));
        singleFadeAnimation();  
    }

    public void clear() {
        m_candle.clearAnimation(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LED_R", getR(ledColor));
        SmartDashboard.putNumber("LED_G", getG(ledColor));
        SmartDashboard.putNumber("LED_B", getB(ledColor));
        SmartDashboard.updateValues();
    }
}
