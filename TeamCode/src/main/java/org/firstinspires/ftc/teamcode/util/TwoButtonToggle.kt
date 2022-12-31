package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys

class TwoButtonToggle(val gamepad: GamepadEx, val key0: GamepadKeys.Button, val key1: GamepadKeys.Button) {
    enum class State {
        A, B
    }
    var state = State.A
    private var stateA = false; private var stateB = false
    private var currentA = false; private var currentB = false
    private var lastA = currentA
    private var lastB = currentB
    fun update() {
        currentA = gamepad.isDown(key0)
        currentB = gamepad.isDown(key1)
        if(currentA && !lastA){
            state = State.A
        }
        if(currentB && !lastB) {
            state = State.B
        }

        lastA = currentA
        lastB = currentB

    }
}