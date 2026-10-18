# host/clock

The AGX boots with its clock at 1970 whenever the carrier board's PMIC RTC
(`nvvrs-pseq-rtc`, `rtc0`) has lost power — it has no backup battery.

`fake-hwclock` is installed, but it does not help on its own: `rtc0` is a kernel module
that registers about a second after `fake-hwclock.service` restores the saved time, and the
kernel (`CONFIG_RTC_HCTOSYS_DEVICE="rtc0"`) immediately sets the system clock from it.
chrony only corrects the clock once NTP is reachable (~80 s after boot on the dock), so every
container and the `rmw-zenohd` router started before that runs with a 1970 time base
(`docker ps` shows `Up 56 years`).

`99-rtc0-fake-hwclock.rules` re-runs `fake-hwclock load` when `rtc0` appears. Without
`FORCE`, `fake-hwclock` only moves the clock forward, so the rule is a no-op when the RTC
is correct.

```bash
bash host/clock/install.sh            # asks for the AGX sudo password via zenity
bash host/clock/install.sh --check
bash host/clock/install.sh --uninstall
```

This is a fallback, not a fix: the restored time is the last hourly/shutdown save, not the
real time. The real fix is a backup battery for the RTC. `rmw-zenohd` additionally waits for
chrony (bounded) before starting — see `robot-net/zenoh/` and `ROBOT-09` in the infra repo.

Containers already running with a wrong clock do not correct themselves; restart them after
chrony has synced.

Verify after a reboot:

```bash
journalctl -b -o short-monotonic -u chrony | grep 'System clock wrong'   # large value = RTC was reset
journalctl -b -o short-monotonic | grep -E 'fake-hwclock|nvvrs-pseq-rtc'
```
