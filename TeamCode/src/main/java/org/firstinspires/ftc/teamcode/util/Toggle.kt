package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys

class Toggle(val gamepad: GamepadEx, val key: GamepadKeys.Button){
    var state = false
    private var currentState = false
    private var lastState = currentState

    fun update() {
        currentState = gamepad.isDown(key)
        if(currentState && !lastState) {
            state = !state
        }
        lastState = currentState
    }

}