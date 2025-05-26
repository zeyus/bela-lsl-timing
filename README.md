# Companion Bela app for liblsl.dart timing tests

[liblsl.dart](https://github.com/zeyus/liblsl.dart) includes a liblsl_timing package that can help you monitor the performance and latency of LSL on your network.

This Bela app will consume streams and record digital inputs to a log file, allowing you to see how performance changes with different network conditions, number of devices, number of streams, frequency, etc.

The digital input monitoring allows you to provide arbitrary external timing events, which could come from a trigger devices, or in the case of our experimental setup, the event start pins come from an FSR sensor on the iPad touch screen to record the moment of user-interaction, and then the event end pins come from a photodiode that displays a visual stimulus once the LSL input has been recieved on a second iPad (and vice-versa).
