# Hello
A Test program for verifying that all the components work

## Motors

### H-Bridge Control: Motor A
| AIN1 | AIN2 | Behavior |
| - | - | - |
| 0 | 0 | Neutral |
| 0 | 1 | Spin CW? |
| 1 | 0 | Spin CCW? |
| 1 | 1 | Brake |

> PWMA Controls motor speed

### H-Bridge Control: Motor B
| BIN1 | BIN2 | Behavior |
| - | - | - |
| 0 | 0 | Neutral |
| 0 | 1 | Spin CW? |
| 1 | 0 | Spin CCW? |
| 1 | 1 | Brake |

> PWMB Controls motor speed


## TODO
- Control the motors
- Read the encoders. Position? Speed? Acceleration? What can we read?
- Read the IMU (Do they need analog pins?)