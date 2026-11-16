"""The battery picked on the dashboard is written to the console log once, as 'BATTERY: <id>', for Scope2429 - and only a choice
made for THIS boot counts: a battery is never carried over from an earlier boot."""
import ntcore

from helpers.battery_id import BATTERY_TOPIC, BOOT_TOPIC, CONFIRM_TOPIC, BatteryId, accept, clean


def test_clean_ids():
    assert clean(' 2026-c ') == '2026-C'
    assert clean('') == '' and clean(None) == ''
    assert clean('x' * 30) == '' and clean('bad;id') == '' and clean('-lead') == ''
    assert clean('Spare 3') == 'SPARE 3'


def test_only_a_choice_for_this_boot_is_accepted():
    assert accept('2026-C@ab12cd34', 'ab12cd34') == '2026-C'
    assert accept('2026-C@00000000', 'ab12cd34') == '', 'sent before the reboot'
    assert accept('2026-C', 'ab12cd34') == '', 'no boot id: an old dashboard, or typed into the NT tree'
    assert accept('', 'ab12cd34') == '' and accept('@ab12cd34', 'ab12cd34') == ''


def _robot():
    inst = ntcore.NetworkTableInstance.create()
    return inst, BatteryId(inst)


def test_logs_each_new_choice_once(capsys):
    inst, b = _robot()
    try:
        pub = inst.get_string_topic(BATTERY_TOPIC).publish()
        boot = inst.get_string_topic(BOOT_TOPIC).subscribe('').get()
        confirm = inst.get_string_topic(CONFIRM_TOPIC).subscribe('')
        assert boot == b.boot and len(boot) == 8
        assert b.poll() == '' and capsys.readouterr().out == '', 'nothing picked: nothing logged'
        pub.set(f'2026-c@{boot}')
        assert b.poll() == '2026-C' and b.poll() == '2026-C'
        assert capsys.readouterr().out == 'BATTERY: 2026-C\n', 'logged once, not every loop'
        assert confirm.get() == '2026-C', 'echoed back for the dashboard'
        pub.set(f'2026-E@{boot}')
        b.poll()
        assert capsys.readouterr().out == 'BATTERY: 2026-E   (was 2026-C)\n'
    finally:
        ntcore.NetworkTableInstance.destroy(inst)


def test_a_new_boot_ignores_the_battery_of_the_last_one(capsys):
    inst = ntcore.NetworkTableInstance.create()
    try:
        pub = inst.get_string_topic(BATTERY_TOPIC).publish()
        first = BatteryId(inst)
        pub.set(f'2026-C@{first.boot}')
        assert first.poll() == '2026-C'
        capsys.readouterr()
        second = BatteryId(inst)                          # the robot program restarts; the dashboard's last value is still there
        assert second.boot != first.boot
        assert second.poll() == '' and second.poll() == '', 'no battery for the new boot until one is picked'
        assert capsys.readouterr().out == '', 'nothing logged from the old boot'
        assert inst.get_string_topic(CONFIRM_TOPIC).subscribe('x').get() == '', 'the dashboard is told: nothing logged yet'
        pub.set(f'2026-D@{second.boot}')
        assert second.poll() == '2026-D'
    finally:
        ntcore.NetworkTableInstance.destroy(inst)
