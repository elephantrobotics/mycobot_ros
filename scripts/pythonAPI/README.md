# ros-python-api

**This is python API for mycobot.**

We support Python2, Python3.5 or later. If you want to use the api, make sure `pyserial` is installed.

```bash
pip2 install pyserial
# or
pip3 install pyserial
```

**Class**:
- [MyCobot](##MyCobot)
- [Angle](##Angle)
- [Coord](##Coord)


## MyCobot

### MyCobot.power_on()

- **Description**

    Robot arm power up.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.power_off()

- **Description**

    Robot arm power down.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.set_free_mode()

- **Description**

    Robot arm into free moving mode.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.get_angles()

- **Description**

    Get the degree of all joints.

- **Parameters**

    None

- **Returns**

    A float list of degree.

### MyCobot.get_angles_by_radian()

- **Description**

    Get the radians of all joints.

- **Parameters**

    None

- **Returns**

    A float list of radian.

### MyCobot.send_angle()

- **Description**

    Send one degree of joint to robot arm.

- **Parameters**

    id: Joint id(common.Angle)

    degree: degree value(float)

    speed: (int)

- **Returns**

    None

- **Example**

    ```python
    from pythonAPI.mycobot import MyCobot
    from pythonAPI.common import Angle


    mycobot = MyCobot()
    mycobot.send_angle(Angle.J2.value, 10, 50)
    ```

### MyCobot.send_angles()

- **Description**

    Send the degrees of all joints to robot arm.

- **Parameters**

    degrees: a list of degree value(List[float])

    speed: (int)

- **Returns**

    None

- **Example**

    ```python
    from pythonAPI.mycobot import MyCobot
    from pythonAPI.common import Angle


    mycobot = MyCobot()
    mycobot.send_angles([0,0,0,0,0,0], 80)
    ```

### MyCobot.send_angles_by_radian()

- **Description**

    Send the radians of all joint to robot arm.

- **Parameters**

    degrees: a list of radian value(List[float])

    speed: (int)

- **Returns**

    None

- **Example**

    ```python
    from pythonAPI.mycobot import MyCobot
    from pythonAPI.common import Angle


    mycobot = MyCobot()
    mycobot.send_angles_by_radian([1,1,1,1,1,1], 70)
    ```

### MyCobot.get_coords()

- **Description**

    Get the Coords from robot arm.

- **Parameters**

    None

- **Returns**

    A float list of coord.

### MyCobot.send_coord()

- **Description**

    Send one coord to robot arm.

- **Parameters**

    id: coord name(common.Coord)

    coord: coord value(float)

    speed: (int)

- **Returns**

    None

- **Example**

    ```python
    from pythonAPI.mycobot import MyCobot
    from pythonAPI.common import Coord


    mycobot = MyCobot()
    mycobot.send_coord(Coord.X.value, -40, 70)
    ```

### MyCobot.send_coords()

- **Description**

    Send all coords to robot arm.

- **Parameters**

    coords: a list of coords value(List[float])

    speed: (int)

- **Returns**

    None

- **Example**

    ```python
    from pythonAPI.mycobot import MyCobot
    from pythonAPI.common import Coord


    mycobot = MyCobot()
    mycobot.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
    ```
### MyCobot.set_color()

- **Description**

    Set the color of the light on the top of the robot arm.


- **Parameters**

    rgb: (string) like: "FF0000"

- **Returns**

    None
### MyCobot.is_moving()

- **Description**

    Judge whether the manipulator is moving or not.

- **Parameters**

    None

- **Returns**

    bool: `True` - moving, `False` - not moving.

### MyCobot.pause()

- **Description**

    Pause movement.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.resume()

- **Description**

    Recovery movement.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.stop()

- **Description**

    Stop moving.

- **Parameters**

    None

- **Returns**

    None

### MyCobot.is_pause()

- **Description**

    Judge whether the manipulator pauses or not.

- **Parameters**

    None

- **Returns**

    bool: `True` - pause, `False` - not pause.

### MyCobot.get_speed()

- **Description**

    Get speed.

- **Parameters**

    None

- **Returns**

    speed: (int)

### MyCobot.set_speed()

- **Description**

    Set speed.

- **Parameters**

    speed: (int)

- **Returns**

    None

## Angle

**Description**

Instance class of joint. It's recommended to use this class to select joint.


## Coord

**Description**

Instance class of coord. It's recommended to use this class to select coord.