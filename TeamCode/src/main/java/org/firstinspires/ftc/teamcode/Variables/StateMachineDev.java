package org.firstinspires.ftc.teamcode.Variables;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.StateTransition;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Event;


public class StateMachineDev implements State, Event{
  
private enum State implements Event {
  T,
  G,
  J
}

public StateMachineDev() {

}

StateMachine() stateMachine = new StateMachine();

stateMachine.addTransition(new StateTransition(State HI, Event T, State G));








}
