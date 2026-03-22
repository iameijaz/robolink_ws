from robolink.models import JointState, LayupSequence


def test_home_is_all_zeros():
    home = JointState.home()
    assert all(v == 0.0 for v in home.to_list())


def test_layup_ready_has_real_values():
    ready = JointState.layup_ready()
    assert ready.j1 == -2.9367  # from real CR5 run at NUST


def test_joint_state_to_list_has_six_values():
    state = JointState(j1=0.1, j2=0.2, j3=0.3, j4=0.4, j5=0.5, j6=0.6)
    assert len(state.to_list()) == 6


def test_layup_sequence_fluent_api():
    seq = (LayupSequence("test")
           .add_waypoint(JointState(j1=0.1))
           .add_waypoint(JointState(j1=0.2)))
    assert len(seq) == 2


def test_layup_sequence_sweep_generates_correct_count():
    seq = LayupSequence.sweep(
        name="sweep_test",
        axis="j2",
        start=-0.5,
        end=0.5,
        steps=6,
    )
    assert len(seq) == 6
    assert seq.waypoints[0].j2 == -0.5
    assert seq.waypoints[-1].j2 == 0.5


def test_layup_sequence_repr():
    seq = LayupSequence("ply_1")
    seq.add_waypoint(JointState())
    assert "ply_1" in repr(seq)
    assert "1 waypoints" in repr(seq)
