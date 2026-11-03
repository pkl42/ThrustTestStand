/**
 * @page system_overview System Overview - Actuator Control Architecture
 *
 * @brief High-level architecture of the actuator and motor control system.
 *
 * This system provides a unified, safety-critical abstraction layer for all
 * actuators (motors, ESCs, and future devices).
 *
 * The architecture is structured into four layers:
 *
 * @section layer1_state Actuator State Layer
 *
 * @ref ActuatorState defines the global lifecycle and safety states for all actuators.
 *
 * Key responsibilities:
 * - Define lifecycle (INIT → READY → ACTIVE → STOPPED)
 * - Define fault handling (ERROR, E_STOP)
 * - Provide system-wide semantic consistency
 *
 *
 * @section layer2_base Base Actuator Layer
 *
 * @ref BaseActuator provides:
 * - State machine enforcement
 * - Emergency stop handling (E-STOP latch)
 * - Common lifecycle API (begin, update, stop)
 *
 * It does NOT implement hardware control.
 *
 *
 * @section layer3_domain Domain Actuators
 *
 * Example: @ref MotorESC
 *
 * Responsibilities:
 * - Implements motor-specific control logic
 * - Adds arming, smoothing, throttling, safety constraints
 * - Translates high-level commands into actuator behavior
 *
 * It depends on BaseActuator for safety and state control.
 *
 *
 * @section layer4_driver Hardware Drivers
 *
 * Example implementations:
 * - PwmEscDriver
 * - DShotEscDriver
 *
 * Responsibilities:
 * - Generate low-level ESC signals
 * - Abstract hardware-specific protocols
 *
 * Drivers are stateless or minimally stateful and owned by domain actuators.
 *
 *
 * @section system_safety Safety Model
 *
 * Safety is enforced at multiple levels:
 *
 * 1. BaseActuator enforces E-STOP (highest priority, latched)
 * 2. MotorESC enforces arming and throttle constraints
 * 3. Drivers only execute validated commands
 *
 * Emergency stop overrides all layers and cannot be bypassed.
 *
 *
 * @section system_dataflow Data Flow
 *
 * @code
 * Application
 *     ↓
 * MotorESC (logic, arming, smoothing)
 *     ↓
 * EscSignalDriver (PWM / DShot)
 *     ↓
 * Hardware (ESC → Motor)
 * @endcode
 *
 *
 * @section system_design_goals Design Goals
 *
 * - Deterministic actuator behavior
 * - Strong safety guarantees (E-STOP always enforced)
 * - Driver abstraction (PWM, DShot, future protocols)
 * - Testability (logic separated from hardware)
 * - Extensibility (new actuators plug into BaseActuator)
 *
 *
 * @section system_threading Threading Model
 *
 * - All actuator APIs must be called from non-ISR context
 * - Only forceStopHardware() is ISR-safe
 *
 *
 * @see ActuatorState
 * @see BaseActuator
 * @see MotorESC
 * @see EscSignalDriver
 */