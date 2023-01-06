package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys

class IntakeToggle(gamepad: GamepadEx, key0: GamepadKeys.Button, key1: GamepadKeys.Button, key2: GamepadKeys.Button) {
    enum class STATE {
        OPEN,
        CLOSE,
        OVERRIDE
    }
    var state: STATE = STATE.OPEN

    fun update() {
        ;
    }
}