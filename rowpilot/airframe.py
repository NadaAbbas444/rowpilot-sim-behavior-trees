"""The drones the ArduPilot simulation can fly: the FireFly 2 (the real drone) and the ArduPilot Iris.

FireFly 2 comes from its SolidWorks design (sw_urdf_exporter, "Drone V3.2 D435"): the body and prop
meshes (simplified by cad_import.py), the rotor positions and the sensor positions are the CAD's.
Body frame = the CAD's base_link: FLU, origin at the bottom of the body, centred.

What the CAD does not settle, and is estimated here (replace with measurements when known):
  - mass and inertia: 0.70 kg body + 4 g per prop, from the CAD parts with typical masses (4 Molicel
    P42A cells 70 g each, Pi 5 with cooler 60 g, 2004 motors 20 g, ...) and 150 g of printed
    structure; the CAD's own figure (0.52 kg) leaves out most components. Weigh the drone.
  - motors and props: 2004 motors with 4-inch props, ~0.7 kg (7 N) thrust each, so thrust/weight ~4.
  - the D435i is tilted 15 deg down (the CAD has it level); the Pi Camera Module 3 is on the left side
    (the CAD has a camera at the same spot on the right).

Rotor model. Like ardupilot_gazebo's Iris: each rotor is a joint driven at ArduPilot's commanded speed
(0-838 rad/s) by a P controller, with two lift-drag "blades" whose drag also yaws the drone. The speed
range and blade radius are the Iris's; the blade area sets the thrust (7 N at full speed). The rotor
inertia is symmetric about the spin axis: the Iris's blade-shaped rotor inertia, spinning at 420 rad/s
on a body this light, shook it at 17-20 m/s^2 and ArduCopter lost attitude control. Inertia and the
speed controller's gain are scaled together (gain * physics step / inertia ~ 1.25, as for the Iris), which
keeps Gazebo's 1 kHz joint control stable. It is a stand-in for the real motors, sized for their thrust.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .sensors import D435i, PiCamera3, SensorRig, rig_model_sdf

MAX_ROTOR_SPEED = 838.0  # rad/s at full throttle (ArduPilot plugin multiplier), as for the Iris
ROTOR_IZZ = 4e-5  # kg m^2 about the spin axis (half that about the others: a disc)
ROTOR_P_GAIN = 0.05  # N m per rad/s of speed error
ROTOR_DAMPING = 1e-4  # N m per rad/s
IMU_LINK_MASS = 0.015  # kg, taken from the body: a much lighter link jitters in Gazebo's physics


@dataclass(frozen=True)
class Rotor:
    name: str
    xyz: tuple[float, float, float]  # body frame, m
    mesh_rpy: tuple[float, float, float]  # prop mesh orientation (from the CAD)
    mesh: str
    channel: int  # ArduPilot motor output (quad X: 1 front right, 2 back left, 3 front left, 4 back right)
    ccw: bool  # spins counter-clockwise seen from above


@dataclass(frozen=True)
class FireFly:
    mass: float = 0.702  # kg, body without props (estimate, see module docstring)
    com: tuple[float, float, float] = (0.001, -0.0007, 0.0377)
    inertia: tuple[float, float, float] = (0.00133, 0.00327, 0.00395)  # ixx iyy izz about the CoM
    size: tuple[float, float, float] = (0.376, 0.234, 0.086)  # body box (prop guards included)
    prop_mass: float = 0.004
    max_thrust_n: float = 7.0  # per motor
    rotors: tuple[Rotor, ...] = (
        Rotor("rotor_0", (0.1245, -0.055, 0.0762), (0.0, 0.0, 1.5493), "prop_front_right.stl", 0, True),
        Rotor("rotor_1", (-0.1245, 0.055, 0.0762), (0.0, 0.0, 1.0042), "prop_back_left.stl", 1, True),
        Rotor("rotor_2", (0.1245, 0.055, 0.0762), (math.pi, 0.0, 0.85733), "prop_front_left.stl", 2, False),
        Rotor("rotor_3", (-0.1245, -0.055, 0.0760), (0.0, 0.0, 1.6802), "prop_back_right.stl", 3, False),
    )

    # Iris rotor stand-in (see module docstring)
    blade_cp: float = 0.084
    blade_cl: float = 4.25 * 0.3  # lift coefficient at the blades' zero-inflow angle of attack

    @property
    def blade_area(self) -> float:
        """Area per blade for max_thrust_n at full speed: T = 2 blades * 0.5 rho (w r)^2 A cl."""
        v = MAX_ROTOR_SPEED * self.blade_cp
        return self.max_thrust_n / (1.2041 * v * v * self.blade_cl)

    @property
    def total_mass(self) -> float:
        return self.mass + len(self.rotors) * self.prop_mass

    def hover_speed(self) -> float:
        """Rotor speed (rad/s) that holds the drone in the air."""
        return MAX_ROTOR_SPEED * math.sqrt(self.total_mass * 9.8 / len(self.rotors) / self.max_thrust_n)


# The real drone's sensors on the FireFly: D435i at the front of the body (CAD: depth module front face
# 6.7 cm ahead of the centre, 3.9 cm up), Pi Camera Module 3 on the left, looking sideways at the crop.
FIREFLY_RIG = SensorRig(
    d435i=D435i(mount_x=0.067, mount_z=0.039, tilt_deg=15.0),
    picam=PiCamera3(pose=(0.0125, 0.062, 0.040, 0.0, 0.0, math.pi / 2)),
)


def _f(*v: float) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def firefly_sdf(ff: FireFly | None = None, model: str = "firefly") -> str:
    """The FireFly 2 airframe for Gazebo with ardupilot_gazebo's ArduPilotPlugin (no cameras: see
    gh_firefly_sdf for the drone with its sensors)."""
    ff = ff or FireFly()
    ixx, iyy, izz = ff.inertia
    sx, sy, sz = ff.size
    surface = (
        "<surface><contact><ode><max_vel>100.0</max_vel><min_depth>0.001</min_depth></ode></contact>"
        "<friction><ode><mu>100000.0</mu><mu2>100000.0</mu2></ode></friction></surface>"
    )
    links = f"""    <link name="base_link">
      <inertial><pose>{_f(*ff.com)} 0 0 0</pose><mass>{ff.mass - IMU_LINK_MASS:g}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="body_collision"><pose>0 0 {sz / 2:.4f} 0 0 0</pose>
        <geometry><box><size>{_f(sx, sy, sz)}</size></box></geometry>{surface}</collision>
      <visual name="body">
        <geometry><mesh><uri>model://{model}/meshes/body.stl</uri></mesh></geometry>
        <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.82 0.82 0.8 1</diffuse>
          <pbr><metal><metalness>0.0</metalness><roughness>0.7</roughness></metal></pbr></material>
      </visual>
    </link>
    <!-- ArduPilot's IMU, at the centre of mass (the plugin's frame conventions: rolled 180 deg) -->
    <link name="imu_link">
      <pose>{_f(*ff.com)} 0 0 0</pose>
      <inertial><mass>{IMU_LINK_MASS}</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>imu_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000.0</update_rate>
      </sensor>
    </link>
    <joint name="imu_joint" type="revolute">
      <child>imu_link</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>0</lower><upper>0</upper><effort>0</effort><velocity>0</velocity></limit>
        <dynamics><damping>1.0</damping></dynamics></axis>
    </joint>
"""
    plugins = ""
    controls = ""
    for r in ff.rotors:
        links += f"""    <link name="{r.name}">
      <pose>{_f(*r.xyz)} 0 0 0</pose>
      <inertial><mass>{ff.prop_mass}</mass>
        <inertia><ixx>{ROTOR_IZZ / 2:g}</ixx><iyy>{ROTOR_IZZ / 2:g}</iyy><izz>{ROTOR_IZZ:g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="prop"><pose>0 0 0 {_f(*r.mesh_rpy)}</pose>
        <geometry><mesh><uri>model://{model}/meshes/{r.mesh}</uri></mesh></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>
    </link>
    <joint name="{r.name}_joint" type="revolute">
      <child>{r.name}</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><damping>{ROTOR_DAMPING:g}</damping></dynamics></axis>
    </joint>
"""
        s = 1 if r.ccw else -1
        for cp in (1, -1):  # two blades; a blade moves along +y at +x when spinning counter-clockwise
            plugins += f"""    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>0.3</a0><alpha_stall>1.4</alpha_stall><cla>4.25</cla><cda>0.10</cda><cma>0.0</cma>
      <cla_stall>-0.025</cla_stall><cda_stall>0.0</cda_stall><cma_stall>0.0</cma_stall>
      <area>{ff.blade_area:.6g}</area><air_density>1.2041</air_density>
      <cp>{cp * ff.blade_cp} 0 0</cp><forward>0 {cp * s} 0</forward><upward>0 0 1</upward>
      <link_name>{r.name}</link_name>
    </plugin>
"""
        plugins += f"""    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>{r.name}_joint</joint_name>
    </plugin>
"""
        controls += f"""      <control channel="{r.channel}">
        <jointName>{r.name}_joint</jointName><useForce>1</useForce>
        <multiplier>{s * MAX_ROTOR_SPEED:g}</multiplier><offset>0</offset>
        <servo_min>1100</servo_min><servo_max>1900</servo_max><type>VELOCITY</type>
        <p_gain>{ROTOR_P_GAIN:g}</p_gain><i_gain>0</i_gain><d_gain>0</d_gain><i_max>0</i_max><i_min>0</i_min>
        <cmd_max>2.5</cmd_max><cmd_min>-2.5</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
"""
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by rowpilot.airframe: edit airframe.py, not this file. -->
<sdf version="1.9">
  <model name="{model}">
{links}    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
{plugins}    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr><fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step><no_time_sync>1</no_time_sync><have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>
{controls}    </plugin>
  </model>
</sdf>
"""


def gh_firefly_sdf(model: str = "gh_firefly") -> str:
    """ArduPilot: the FireFly 2 with the real drone's sensors."""
    # The D435i's mass is in the FireFly's body estimate already: the sensor link only gets a token mass.
    return rig_model_sdf(model, "model://firefly", "base_link", FIREFLY_RIG, "ArduPilot SITL, FireFly 2.", 0.005)


@dataclass(frozen=True)
class Vehicle:
    """What the scripts need to know about a simulated drone."""

    model: str  # Gazebo model name, also the sensor topic prefix (/<model>/...)
    rig: SensorRig = field(default_factory=SensorRig)
    home_z: float = 0.0  # body frame origin above the floor at rest (m)
    frame_parm: str = ""  # ArduPilot parameters for the airframe, after copter.parm


VEHICLES = {
    "firefly": Vehicle("gh_firefly", FIREFLY_RIG, 0.0, "firefly.parm"),
    "iris": Vehicle("gh_iris", SensorRig(), 0.195, "gazebo-iris.parm"),
}
