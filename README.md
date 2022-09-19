# ROS2 QOS

This repository demonstrates the usage of ROS2 quality of service (QoS). It is an advanced behavior of publish and subscribe (pub/sub) communications.

![image](resources/qos1.png)

QoS is a DDS concepts which has been formalized into ROS2. Additionally, non-DDS RMWs can choose to provide QoS.

If QoS profile matches for both publisher and subscriber, the messages will start flowing in. Otherwise, there will be no message coming in as the system is unable to guarantee the QoS.

![image](resources/qos2.png)

## Types of QoS Policies

- History: How many messages to keep locally? Similar to ROS1 "queue_size".
    - Legal Values:
        - `KEEP_ALL`:
        - `KEEP_LAST + depth`:
    - Compatibility: N/A - does not apply to matching.
    - Example: Image processing queue.
- Durability: Should publishers provide old messages? Similar to ROS1 "latching".
    - Legal Values:
        - `VOLATILE`: Late joining subscriptions receive nothing.
        - `TRANSIENT_LOCAL`: Publisher provide old messages.
    - Compatibility: `TRANSIENT_LOCAL` > `VOLATILE`
    - Example: Latest mission state machine.
- Reliability: Do messages have to be deliver/received?
    - Legal Values:
        - `BEST_EFFORT`: No delivery guarantee.
        - `RELIABLE`: Guarantee delivery*
    - Compatibility: `RELIABLE` > `BEST_EFFORT`
    - Example: Visualizer for human dosen't need retry; safety critical update must get through.
- Lifespan: How long before an un-sent/un-consume message is not useful anymore?
    - Legal Values:
        - `Duration`: duration.
    - Compatibility: Offered duration >= Requested duration
    - Example: Estimate pose when moving.
- Deadline: How often must I send messages? (minimum frequency). If it is being trigger both the subscriber and publisher will get an event callback.
    - Legal Values:
        - `Period`: duration.
    - Compatibility: Offered duration <= Requested duration
    - Callbacks:
        - Publisher - OfferedDeadlineMissed
        - Subscription - RequestedDeadlineMissed
    - Example: `/cmd_vel` safety watchdog.


![image](resources/qos_history.png)
![image](resources/qos_durability.png)
![image](resources/qos_reliability.png)


## New Concepts

- Some QoS policies generate `events`.
- Subscription already had `message callback` and now there is a `QoS Event callback`.
- Publishers get them too!
