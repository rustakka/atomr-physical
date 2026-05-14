"""Smoke tests for the atomr-physical Python overlay.

These exercise the native extension end to end — value construction,
field round-trips, the safety envelope, and the offline ROS2 topic
plan. Run after building the extension::

    maturin develop -m crates/py-bindings/Cargo.toml
    pytest python/atomr_physical/tests
"""

import pytest

import atomr_physical as ap
from atomr_physical.errors import OutOfRange


def test_quantity_round_trips():
    q = ap.Quantity(1.5, "rad")
    assert q.value == 1.5
    assert q.unit == "rad"


def test_reading_carries_sensor_and_frame():
    r = ap.Reading("s1", 21.0, "C", frame="base_link")
    assert r.sensor == "s1"
    assert r.value == 21.0
    assert r.unit == "C"
    assert r.frame == "base_link"
    assert r.timestamp_ms > 0


def test_command_and_ack():
    cmd = ap.Command("a1", setpoint=0.5, mode="position", unit="rad")
    assert cmd.actuator == "a1"
    assert cmd.mode == "position"
    assert cmd.setpoint == 0.5

    ack = ap.CommandAck.accepted("a1")
    assert ack.accepted_flag is True

    rej = ap.CommandAck.rejected("a1", "estop engaged")
    assert rej.accepted_flag is False
    assert rej.detail == "estop engaged"


def test_calibration_is_linear():
    cal = ap.Calibration(scale=2.0, offset=1.0)
    assert cal.apply(3.0) == 7.0


def test_sampling_policy_variants():
    rate = ap.SamplingPolicy.fixed_rate(100)
    assert rate.period_ms == 100
    assert rate.is_on_demand is False

    on_demand = ap.SamplingPolicy.on_demand()
    assert on_demand.period_ms is None
    assert on_demand.is_on_demand is True


def test_safety_envelope_clamps_and_rejects():
    clamping = ap.SafetyEnvelope.clamping(0.0, 1.0)
    assert clamping.enforce("a1", 5.0) == 1.0

    rejecting = ap.SafetyEnvelope.rejecting(0.0, 1.0)
    assert rejecting.enforce("a1", 0.5) == 0.5
    with pytest.raises(OutOfRange):
        rejecting.enforce("a1", 5.0)


def test_robot_model_collects_joints():
    model = ap.RobotModel()
    model.add_joint(ap.Joint("j1", "shoulder_pan", actuator="a1", feedback="s1"))
    model.add_joint(ap.Joint("j2", "shoulder_lift", actuator="a2"))
    model.add_auxiliary_sensor("imu0")
    assert model.joint_count == 2
    assert model.joint_ids == ["j1", "j2"]
    assert model.auxiliary_sensor_ids == ["imu0"]


def test_ros2_topic_map_binds_both_directions():
    topics = ap.TopicMap()
    topics.bind_sensor(
        "s1", ap.Ros2Endpoint.publish("/robot/temp", "sensor_msgs/msg/Temperature")
    )
    topics.bind_actuator(
        "a1", ap.Ros2Endpoint.subscribe("/robot/cmd", "std_msgs/msg/Float64")
    )
    assert topics.len == 2
    assert topics.sensor_endpoint("s1").direction == "publish"
    assert topics.actuator_endpoint("a1").direction == "subscribe"
