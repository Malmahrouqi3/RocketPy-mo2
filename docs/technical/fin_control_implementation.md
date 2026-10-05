The flight loop confirms the feedback path: sampled controllers run at time nodes before integration, continuous controllers run after each accepted solver step, and `post_step_callback(flight)` is already available for a full-lifecycle observer. I’ll now consolidate this into a concrete API recommendation, including the cache/geometry update that must accompany fin commands and a callback design for shared controller inputs.

The repository already has most of the control plumbing:

- [controller.py](rocketpy/control/controller.py#L8) supports arbitrary interactive objects, state, history, sensors, and environment.
- [flight.py](rocketpy/simulation/flight.py#L863) measures sensors and invokes controllers at scheduled nodes.
- Continuous controllers run after each accepted solver step in [flight.py](rocketpy/simulation/flight.py#L783).
- Individual fins already exist and are registered as aerodynamic surfaces.
- The existing `cant_angle` setter updates fin geometry and rotation, but the rocket’s cached center-of-pressure map is not refreshed, as noted in [rocket.py](rocketpy/rocket/rocket.py#L704).

**Recommended implementation**

Add a small `FinActuator` adapter rather than putting controller state directly on `Fin`:

```python
class FinActuator:
    def __init__(self, rocket, fin, min_angle=-20.0, max_angle=20.0):
        self.rocket = rocket
        self.fin = fin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.command_angle = fin.cant_angle

    @property
    def angle(self):
        return self.fin.cant_angle

    def set_command(self, angle):
        self.command_angle = max(
            self.min_angle,
            min(self.max_angle, float(angle)),
        )
        self.fin.cant_angle = self.command_angle
        self.rocket.evaluate_surfaces_cp_to_cdm()

    def _reset(self):
        self.set_command(0.0)
```

Then use one controller for coordinated control of all fins:

```python
def fin_controller(
    time,
    sampling_rate,
    state,
    state_history,
    observed_variables,
    actuators,
    sensors,
    environment,
):
    pitch_rate = state[10]
    yaw_rate = state[11]

    pitch_command = -2.0 * pitch_rate
    yaw_command = -2.0 * yaw_rate

    actuators[0].set_command(+pitch_command + yaw_command)
    actuators[1].set_command(+pitch_command - yaw_command)
    actuators[2].set_command(-pitch_command - yaw_command)
    actuators[3].set_command(-pitch_command + yaw_command)

    return {
        "time": time,
        "angles": [actuator.angle for actuator in actuators],
        "pitch_rate": pitch_rate,
        "yaw_rate": yaw_rate,
    }
```

The controller should receive:

```python
actuators = [
    FinActuator(rocket, fin_1),
    FinActuator(rocket, fin_2),
    FinActuator(rocket, fin_3),
    FinActuator(rocket, fin_4),
]
```

A single controller is preferable when the fins participate in the same attitude-control law. Use one controller per fin only when the fins have genuinely independent control loops.

**Feedback callback**

Extend `_Controller` with an optional callback executed immediately before the controller function:

```python
def __call__(self, time, state_vector, state_history, sensors, environment):
    if self.input_callback is not None:
        self.input_callback(
            time,
            state_vector,
            state_history,
            self.interactive_objects,
            sensors,
            environment,
        )

    observed_variables = self.controller_function(...)
```

The callback should mutate a shared input object, for example:

```python
def update_controller_inputs(
    time, state, history, actuators, sensors, environment
):
    inputs = actuators[0].controller_inputs
    inputs["altitude"] = state[2]
    inputs["vertical_velocity"] = state[5]
    inputs["gyro"] = sensors[0].measurement
```

This preserves the current controller signature and lets the controller react to updated state, sensor data, or environment values. The state is already available directly, so a callback is mainly useful for deriving and sharing additional controller inputs.

**Important details**

1. Refresh `rocket.evaluate_surfaces_cp_to_cdm()` whenever a fin angle changes.
2. Reset all fin actuators in `Flight.__init_controllers`, alongside the existing air-brake reset.
3. Add a public `Rocket.add_fin_controller(...)` convenience method instead of requiring users to call private `_add_controllers`.
4. Consider actuator rate limits later if servo dynamics matter. The first version can model instantaneous angle commands.
5. Add focused tests for:
   - independent fin commands,
   - angle clamping,
   - controller callback ordering,
   - CP-map refresh after fin movement,
   - actuator reset between flights.

This design fits the current RocketPy controller architecture while allowing both one-controller-for-all-fins and independently controlled fins.
