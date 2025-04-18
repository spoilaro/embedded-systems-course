# Notes

Some possible bugs:
- In fault state the movement led blinks 3 times too many
- When returning from the emergency state, melody plays when doing anything that calls send_state() (?)