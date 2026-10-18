#!/usr/bin/env bash
# Install the rtc0 -> fake-hwclock udev rule on the AGX.
#   bash host/clock/install.sh             # install
#   bash host/clock/install.sh --check     # read-only
#   bash host/clock/install.sh --uninstall
set -euo pipefail

AGX="${AGX:-systemlabagx@10.10.12.3}"
RULE=99-rtc0-fake-hwclock.rules
DEST=/etc/udev/rules.d/${RULE}
HERE="$(cd "$(dirname "$0")" && pwd)"

remote_sudo() {
  { zenity --password --title="sudo on ${AGX}"; printf '%s\n' "$1"; } \
    | ssh "${AGX}" 'sudo -S -p "" bash -s'
}

case "${1:-}" in
  --check)
    ssh "${AGX}" "set -e
      if cmp -s - ${DEST}; then echo 'rule: up to date'; else echo 'rule: missing or differs'; exit 1; fi" < "${HERE}/${RULE}"
    ;;
  --uninstall)
    remote_sudo "rm -f ${DEST} && udevadm control --reload && echo removed"
    ;;
  "")
    command -v zenity >/dev/null || { echo "zenity is required" >&2; exit 1; }
    remote_sudo "set -e
      command -v fake-hwclock >/dev/null || { echo 'fake-hwclock is not installed' >&2; exit 1; }
      cat > ${DEST} <<'RULE_EOF'
$(cat "${HERE}/${RULE}")
RULE_EOF
      chmod 644 ${DEST}
      udevadm control --reload
      udevadm test /sys/class/rtc/rtc0 2>&1 | grep -q 'fake-hwclock load' && echo 'installed; udev matches rtc0'"
    ;;
  *) echo "usage: $0 [--check|--uninstall]" >&2; exit 2 ;;
esac
