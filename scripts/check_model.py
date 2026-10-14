"""Check the integrated model with: python scripts/check_model.py."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def inertia_tensor(model, body_id):
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body_id])
    rotation = rotation.reshape(3, 3)
    return rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T


def main():
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "bimanualHand/openarm_hand_bimanual.xml")
    )
    reference = mujoco.MjModel.from_xml_path(str(ROOT / "openarm/MJCF_bimanual.xml"))
    urdf = ET.parse(ROOT / "bimanualHand/openarm_hand_bimanual.urdf").getroot()
    xml = ET.parse(ROOT / "bimanualHand/openarm_hand_bimanual.xml").getroot()
    assert (model.nq, model.nv, model.nu) == (54, 54, 54)
    assert (model.ngeom, model.nexclude) == (132, 108)
    np.testing.assert_allclose(model.opt.gravity, [0, 0, -9.81])

    for side in ("left", "right"):
        for suffix in ["base_link", "link1", "link2", "link3", "link4"]:
            name = f"openarm_{side}_{suffix}"
            target_id, source_id = model.body(name).id, reference.body(name).id
            np.testing.assert_allclose(model.body_mass[target_id], reference.body_mass[source_id])
            np.testing.assert_allclose(model.body_ipos[target_id], reference.body_ipos[source_id])
            tensor = inertia_tensor(model, target_id)
            np.testing.assert_allclose(tensor, inertia_tensor(reference, source_id))
            item = urdf.find(f"link[@name='{name}']/inertial")
            assert float(item.find("mass").get("value")) == model.body_mass[target_id]
            np.testing.assert_allclose(
                np.fromstring(item.find("origin").get("xyz"), sep=" "), model.body_ipos[target_id]
            )
            values = {key: float(value) for key, value in item.find("inertia").attrib.items()}
            urdf_tensor = np.array([
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ])
            np.testing.assert_allclose(tensor, urdf_tensor, atol=1e-14)
        output_name = f"openarm_{side}_link5"
        output = model.body(output_name).id
        flange_mass, flange_radius, flange_thickness = 0.05, 0.056 / 2 * (2 / 3), 0.006
        disk_inertia = flange_mass * flange_radius**2 * np.array([0.25, 0.25, 0.5])
        disk_inertia[:2] += flange_mass * flange_thickness**2 / 12
        assert model.body_mass[output] == flange_mass
        np.testing.assert_allclose(model.body_ipos[output], [0, 0, 0])
        np.testing.assert_allclose(inertia_tensor(model, output), np.diag(disk_inertia))
        assert xml.find(f".//body[@name='{output_name}']/geom") is None
        flange = urdf.find(f"link[@name='{output_name}']/inertial")
        assert float(flange.find("mass").get("value")) == flange_mass
        np.testing.assert_allclose(np.fromstring(flange.find("origin").get("xyz"), sep=" "), [0, 0, 0])
        tensor = flange.find("inertia")
        np.testing.assert_allclose([float(tensor.get(axis)) for axis in ("ixx", "iyy", "izz")], disk_inertia)
        assert all(float(tensor.get(axis)) == 0 for axis in ("ixy", "ixz", "iyz"))
        hand_name = f"HOUSING-{side[0].upper()}"
        housing_id = model.body(hand_name).id
        assert model.body_mass[housing_id] == 1.8
        original_hand = ET.parse(ROOT / f"Hand/Hand_{side}/MJCF_{side[0].upper()}.xml").getroot()
        original_housing = original_hand.find(f".//body[@name='{hand_name}']/inertial")
        np.testing.assert_allclose(model.body_ipos[housing_id], np.fromstring(original_housing.get("pos"), sep=" "))
        expected = np.fromstring(original_housing.get("fullinertia"), sep=" ") * (1.8 / float(original_housing.get("mass")))
        expected_tensor = np.array([
            [expected[0], expected[3], expected[4]],
            [expected[3], expected[1], expected[5]],
            [expected[4], expected[5], expected[2]],
        ])
        # Allow the small reconstruction residual of MuJoCo's fullinertia eigensolver.
        np.testing.assert_allclose(inertia_tensor(model, housing_id), expected_tensor, atol=1e-10)
        housing = urdf.find(f"link[@name='{hand_name}']/inertial")
        assert float(housing.find("mass").get("value")) == 1.8
        np.testing.assert_allclose(np.fromstring(housing.find("origin").get("xyz"), sep=" "), model.body_ipos[housing_id])
        np.testing.assert_allclose([float(housing.find("inertia").get(axis)) for axis in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")], expected, atol=1e-14)
        np.testing.assert_allclose(model.body_mass[model.body_weldid == output].sum(), flange_mass + 1.8)

    joints = model.actuator_trnid[:, 0]
    qpos_ids = model.jnt_qposadr[joints]
    assert model.actuator_forcelimited.all()
    for actuator in range(10, model.nu):
        name = model.joint(int(joints[actuator])).name
        effort = float(urdf.find(f"joint[@name='{name}']/limit").get("effort"))
        np.testing.assert_allclose(model.actuator_forcerange[actuator], [-effort, effort])

    data = mujoco.MjData(model)
    for pose in ("zero", "home"):
        if pose == "home":
            mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        else:
            mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        assert data.ncon == 0, f"initial contacts in {pose}"
        np.testing.assert_allclose(data.ctrl, data.qpos[qpos_ids])
        for _ in range(5000):
            mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        assert not data.warning.number.any(), f"solver warning in {pose}"
        assert abs(data.time - 5.0) < 1e-9, "unexpected simulation reset"
        assert np.max(np.abs(data.qvel)) < 1e-3, f"did not settle in {pose}"
        error = np.rad2deg(np.abs(data.qpos[qpos_ids[:10]] - data.ctrl[:10])).max()
        assert error < 3, f"excessive arm drift in {pose}: {error:.3f} deg"
        print(f"{pose}: 5 s stable, maximum arm tracking error {error:.3f} deg")

    # Prove that wrist/finger servos saturate under a large position error.
    for column in (0, 1):
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        data.ctrl[10:] = model.actuator_ctrlrange[10:, column]
        mujoco.mj_forward(model, data)
        assert np.all(data.actuator_force >= model.actuator_forcerange[:, 0] - 1e-9)
        assert np.all(data.actuator_force <= model.actuator_forcerange[:, 1] + 1e-9)
        assert np.any(np.isclose(
            np.abs(data.actuator_force[10:]), model.actuator_forcerange[10:, 1]
        )), "torque saturation was not exercised"
    print("PASS: inertia tensors, fixed-child payload, home mapping and torque limits")


if __name__ == "__main__":
    main()
