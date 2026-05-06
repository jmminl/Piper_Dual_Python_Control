import time
import json

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
)

MOTION_FILE = "leader_motion.json"


def safe_disconnect(robot):
    if robot is None:
        return

    try:
        if hasattr(robot, "disconnect"):
            robot.disconnect()
            print("Robot disconnected")
        elif hasattr(robot, "close"):
            robot.close()
            print("Robot closed")
        elif hasattr(robot, "shutdown"):
            robot.shutdown()
            print("Robot shutdown")
    except Exception as e:
        print("Disconnect failed:", e)

    time.sleep(1.0)


def wait_enable(robot, timeout=10.0):
    print("Robot enable start")
    start_t = time.time()

    while True:
        ok = robot.enable()

        if ok:
            print(robot.get_arm_status())
            print("Enable The Robot")
            return True

        if time.time() - start_t > timeout:
            raise RuntimeError("Robot enable timeout")

        print("Waiting robot enable...")
        time.sleep(0.5)


def create_robot(interface, channel, enable_robot=False):
    robot_cfg = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.DEFAULT,
        interface=interface,
        channel=channel,
    )

    robot = AgxArmFactory.create_arm(robot_cfg)

    try:
        print(f"Robot connect start: {interface}, {channel}")
        robot.connect()
        print("Robot connect done")

        end_effector = robot.init_effector(
            robot.OPTIONS.EFFECTOR.AGX_GRIPPER
        )

        time.sleep(0.5)

        if enable_robot:
            wait_enable(robot)
        else:
            print("Connect Only - Enable Skip")

        return robot, end_effector

    except Exception as e:
        safe_disconnect(robot)
        raise e


def extract_joint_list(joint_obj):
    if joint_obj is None:
        return None

    if isinstance(joint_obj, (list, tuple)):
        return list(joint_obj)

    if hasattr(joint_obj, "tolist"):
        try:
            return joint_obj.tolist()
        except Exception:
            pass

    if hasattr(joint_obj, "msg"):
        msg = joint_obj.msg

        if isinstance(msg, (list, tuple)):
            return list(msg)

        if hasattr(msg, "tolist"):
            try:
                return msg.tolist()
            except Exception:
                pass

        candidate_fields = [
            "joint",
            "joints",
            "joint_angles",
            "angle",
            "angles",
            "position",
            "positions",
            "data",
        ]

        for field in candidate_fields:
            if hasattr(msg, field):
                value = getattr(msg, field)

                if isinstance(value, (list, tuple)):
                    return list(value)

                if hasattr(value, "tolist"):
                    try:
                        return value.tolist()
                    except Exception:
                        pass

        joint_fields = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        if all(hasattr(msg, field) for field in joint_fields):
            return [getattr(msg, field) for field in joint_fields]

    try:
        return list(joint_obj)
    except Exception:
        print("[DEBUG] Cannot extract joint:", joint_obj)
        return None


def read_leader_joint(robot):
    joint_obj = robot.get_leader_joint_angles()

    if joint_obj is None:
        joint_obj = robot.get_joint_angles()

    joint = extract_joint_list(joint_obj)

    if joint is None or len(joint) < 6:
        return None

    return joint[:6]


def joint_diff_max(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def interpolate_joint(prev_joint, target_joint, steps):
    result = []

    for i in range(1, steps + 1):
        ratio = i / steps

        joint = [
            prev_joint[j] + (target_joint[j] - prev_joint[j]) * ratio
            for j in range(6)
        ]

        result.append(joint)

    return result


def dual_leader_to_follower_live(
    robot,
    interval=0.02,
    threshold=0.001,
):
    print("===================================")
    print("Dual Live Mode")
    print("Leader 움직임을 Follower가 실시간 추종")
    print("Ctrl + C 종료")
    print("===================================")

    robot.set_leader_mode()
    print("Leader Mode Set")

    time.sleep(1.0)

    robot.set_follower_mode()
    print("Follower Mode Set")

    time.sleep(1.0)

    wait_enable(robot)

    prev_joint = None

    while True:
        leader_joint = read_leader_joint(robot)

        if leader_joint is None:
            print("[WARN] Leader joint read failed")
            time.sleep(interval)
            continue

        if prev_joint is not None:
            diff = joint_diff_max(leader_joint, prev_joint)

            if diff < threshold:
                time.sleep(interval)
                continue

        print("[LIVE]")
        print("Leader Joint:", leader_joint)

        try:
            robot.move_j(leader_joint)
        except Exception as e:
            print("[WARN] Follower move_j failed:", e)

        prev_joint = leader_joint
        time.sleep(interval)


def record_leader_motion(
    robot,
    duration=10.0,
    interval=0.02,
    save_path=MOTION_FILE,
):
    print("===================================")
    print("Leader Motion Recording Start")
    print("팔을 직접 움직이세요.")
    print(f"Duration: {duration} sec")
    print(f"Interval: {interval} sec")
    print("===================================")

    robot.set_leader_mode()
    print("Leader Mode Set")

    time.sleep(1.0)

    motion_data = []
    start_time = time.time()

    while time.time() - start_time < duration:
        t = time.time() - start_time

        joint = read_leader_joint(robot)

        if joint is None:
            print("[WARN] joint read failed")
            time.sleep(interval)
            continue

        motion_data.append({
            "time": t,
            "joint": joint,
        })

        print(f"[REC] t={t:.3f}")
        print("Joint:", joint)

        time.sleep(interval)

    if len(motion_data) == 0:
        raise RuntimeError("No motion data recorded")

    with open(save_path, "w") as f:
        json.dump(motion_data, f, indent=4)

    print("===================================")
    print(f"Motion saved: {save_path}")
    print(f"Saved Point Count: {len(motion_data)}")
    print("===================================")


def replay_motion_one_follower(
    robot,
    load_path=MOTION_FILE,
    threshold=0.001,
    speed_scale=1.0,
    command_interval=0.02,
):
    print("===================================")
    print("Replay One Follower")
    print("Interpolated Replay")
    print(f"Speed Scale: {speed_scale}")
    print(f"Command Interval: {command_interval}")
    print("===================================")

    if speed_scale <= 0:
        speed_scale = 1.0

    robot.set_follower_mode()
    print("Follower Mode Set")

    time.sleep(1.0)
    wait_enable(robot)

    with open(load_path, "r") as f:
        motion_data = json.load(f)

    if len(motion_data) == 0:
        raise RuntimeError("Motion file is empty")

    prev_joint = None
    prev_time = None

    for idx, data in enumerate(motion_data):
        current_time = data["time"]
        joint = data["joint"][:6]

        if prev_joint is None:
            print(f"[PLAY {idx}] t={current_time:.3f}")
            print("Joint:", joint)

            try:
                robot.move_j(joint)
            except Exception as e:
                print("[WARN] move_j failed:", e)

            prev_joint = joint
            prev_time = current_time
            time.sleep(command_interval)
            continue

        diff = joint_diff_max(joint, prev_joint)

        if diff < threshold:
            prev_time = current_time
            continue

        dt = current_time - prev_time

        if dt <= 0:
            dt = command_interval

        replay_dt = dt / speed_scale
        steps = max(1, int(replay_dt / command_interval))

        interpolated_joints = interpolate_joint(
            prev_joint=prev_joint,
            target_joint=joint,
            steps=steps,
        )

        print(f"[PLAY {idx}] t={current_time:.3f}, steps={steps}")
        print("Target Joint:", joint)

        for interp_joint in interpolated_joints:
            try:
                robot.move_j(interp_joint)
            except Exception as e:
                print("[WARN] move_j failed:", e)
                break

            time.sleep(command_interval)

        prev_joint = joint
        prev_time = current_time

    print("Replay Done")


def replay_motion_two_followers(
    robot_left,
    robot_right,
    load_path=MOTION_FILE,
    threshold=0.001,
    speed_scale=1.0,
    command_interval=0.02,
):
    print("===================================")
    print("Replay Two Followers")
    print("Interpolated Dual Replay")
    print(f"Speed Scale: {speed_scale}")
    print(f"Command Interval: {command_interval}")
    print("===================================")

    if speed_scale <= 0:
        speed_scale = 1.0

    robot_left.set_follower_mode()
    robot_right.set_follower_mode()

    print("Left Follower Mode Set")
    print("Right Follower Mode Set")

    time.sleep(1.0)

    wait_enable(robot_left)
    wait_enable(robot_right)

    with open(load_path, "r") as f:
        motion_data = json.load(f)

    if len(motion_data) == 0:
        raise RuntimeError("Motion file is empty")

    prev_joint = None
    prev_time = None

    for idx, data in enumerate(motion_data):
        current_time = data["time"]
        joint = data["joint"][:6]

        if prev_joint is None:
            print(f"[DUAL PLAY {idx}] t={current_time:.3f}")
            print("Joint:", joint)

            try:
                robot_left.move_j(joint)
            except Exception as e:
                print("[WARN] left move_j failed:", e)

            try:
                robot_right.move_j(joint)
            except Exception as e:
                print("[WARN] right move_j failed:", e)

            prev_joint = joint
            prev_time = current_time
            time.sleep(command_interval)
            continue

        diff = joint_diff_max(joint, prev_joint)

        if diff < threshold:
            prev_time = current_time
            continue

        dt = current_time - prev_time

        if dt <= 0:
            dt = command_interval

        replay_dt = dt / speed_scale
        steps = max(1, int(replay_dt / command_interval))

        interpolated_joints = interpolate_joint(
            prev_joint=prev_joint,
            target_joint=joint,
            steps=steps,
        )

        print(f"[DUAL PLAY {idx}] t={current_time:.3f}, steps={steps}")
        print("Target Joint:", joint)

        for interp_joint in interpolated_joints:
            try:
                robot_left.move_j(interp_joint)
            except Exception as e:
                print("[WARN] left move_j failed:", e)

            try:
                robot_right.move_j(interp_joint)
            except Exception as e:
                print("[WARN] right move_j failed:", e)

            time.sleep(command_interval)

        prev_joint = joint
        prev_time = current_time

    print("Dual Replay Done")


def select_arm_mode():
    while True:
        arm_mode = input("Arm Mode? leader / follower : ").strip().lower()

        if arm_mode in ["leader", "follower"]:
            return arm_mode

        print("Wrong Arm Mode")


def change_arm_mode(robot):
    print("===================================")
    print("Change Piper Arm Mode")
    print("===================================")

    arm_mode = select_arm_mode()

    if arm_mode == "leader":
        robot.set_leader_mode()
        print("Leader Mode Set")

    elif arm_mode == "follower":
        robot.set_follower_mode()
        print("Follower Mode Set")

    time.sleep(1.0)

    print("===================================")
    print("Mode Setting Complete")
    print("팔 전원을 껐다 켠 뒤 다시 실행하세요.")
    print("===================================")


def select_os():
    while True:
        os_type = input("OS Type? window / ubuntu : ").strip().lower()

        if os_type in ["window", "windows"]:
            return "agx_cando"

        elif os_type in ["ubuntu", "linux"]:
            return "socketcan"

        else:
            print("Wrong OS Type")


def select_channel(interface):
    if interface == "agx_cando":
        return input("CAN channel? ex) 0 / 1 : ").strip()

    elif interface == "socketcan":
        return input("CAN channel? ex) can0 / can1 : ").strip()


def select_speed_scale():
    while True:
        value = input("Speed Scale? 1.0 normal / 0.5 slow / 2.0 fast : ").strip()

        try:
            speed_scale = float(value)

            if speed_scale <= 0:
                print("Speed Scale must be greater than 0")
                continue

            return speed_scale

        except ValueError:
            print("Wrong Speed Scale")


def select_mode():
    print()
    print("Mode Select")
    print("1 : Dual Live - Leader 움직임을 Follower가 실시간 추종")
    print("2 : Record - Leader 모션 저장")
    print("3 : Replay One - Follower 1개 저장 모션 재생")
    print("4 : Replay Two - Follower 2개 저장 모션 동시 재생")
    print("5 : Change Mode - 팔을 Leader / Follower 모드로 설정")
    print()

    while True:
        mode = input("Select Mode 1 / 2 / 3 / 4 / 5 : ").strip()

        if mode in ["1", "2", "3", "4", "5"]:
            return mode

        print("Wrong Mode")


if __name__ == "__main__":
    robot = None
    robot_left = None
    robot_right = None

    try:
        interface = select_os()
        mode = select_mode()

        if mode == "1":
            channel = select_channel(interface)

            robot, _ = create_robot(
                interface=interface,
                channel=channel,
                enable_robot=False,
            )

            dual_leader_to_follower_live(
                robot=robot,
                interval=0.02,
                threshold=0.001,
            )

        elif mode == "2":
            channel = select_channel(interface)

            robot, _ = create_robot(
                interface=interface,
                channel=channel,
                enable_robot=False,
            )

            record_leader_motion(
                robot=robot,
                duration=10.0,
                interval=0.02,
                save_path=MOTION_FILE,
            )

        elif mode == "3":
            channel = select_channel(interface)
            speed_scale = select_speed_scale()

            robot, _ = create_robot(
                interface=interface,
                channel=channel,
                enable_robot=False,
            )

            replay_motion_one_follower(
                robot=robot,
                load_path=MOTION_FILE,
                threshold=0.001,
                speed_scale=speed_scale,
                command_interval=0.02,
            )

        elif mode == "4":
            speed_scale = select_speed_scale()

            print("Left Follower Channel")
            left_channel = select_channel(interface)

            print("Right Follower Channel")
            right_channel = select_channel(interface)

            robot_left, _ = create_robot(
                interface=interface,
                channel=left_channel,
                enable_robot=False,
            )

            robot_right, _ = create_robot(
                interface=interface,
                channel=right_channel,
                enable_robot=False,
            )

            replay_motion_two_followers(
                robot_left=robot_left,
                robot_right=robot_right,
                load_path=MOTION_FILE,
                threshold=0.001,
                speed_scale=speed_scale,
                command_interval=0.02,
            )

        elif mode == "5":
            channel = select_channel(interface)

            robot, _ = create_robot(
                interface=interface,
                channel=channel,
                enable_robot=False,
            )

            change_arm_mode(robot)

    except KeyboardInterrupt:
        print("stop")

    except Exception as e:
        print("[ERROR]", e)

    finally:
        safe_disconnect(robot)
        safe_disconnect(robot_left)
        safe_disconnect(robot_right)