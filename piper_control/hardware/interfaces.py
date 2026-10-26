"""Resolve configured arm identities to the current SocketCAN interface names.

USB serials identify adapters; USB paths identify physical ports. Discovery only
reads sysfs and never brings an interface up or sends CAN frames.
"""

import copy
from pathlib import Path
import re

_USB_DEVICE = re.compile(r"\d+-\d+(?:\.\d+)*")
_USB_INTERFACE = re.compile(r"\d+-\d+(?:\.\d+)*:\d+\.\d+")
_CHANNEL = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,14}")


def _read(path):
    try:
        return path.read_text().strip() or None
    except OSError:
        return None


def discover_interfaces(sysfs_root="/sys/class/net"):
    """Return CAN names and their nearest USB device/interface identities.

    An adapter without a serial remains serial-less: an upstream hub's serial
    must not be mistaken for the adapter's identity. Non-USB CAN devices are
    included with null USB fields and can use an explicit channel config.
    """
    interfaces = []
    for net in sorted(Path(sysfs_root).iterdir()):
        if _read(net / "type") != "280":  # Linux ARPHRD_CAN
            continue
        device = (net / "device").resolve()
        ancestors = (device, *device.parents)
        usb_device = next(
            (path for path in ancestors if _USB_DEVICE.fullmatch(path.name)), None
        )
        usb_interface = next(
            (path.name for path in ancestors if _USB_INTERFACE.fullmatch(path.name)),
            None,
        )
        interfaces.append(
            dict(
                channel=net.name,
                usb_serial=_read(usb_device / "serial") if usb_device else None,
                usb_path=usb_device.name if usb_device else None,
                usb_interface=usb_interface,
            )
        )
    return interfaces


def configured_devices(config):
    """Yield (side, role, spec) for selected followers and optional leaders."""
    for side in config["arms"]:
        pair = config["pairs"][side]
        if not isinstance(pair.get("follower"), dict):
            raise ValueError("robot.pairs.{}.follower is required".format(side))
        for role in ("follower", "leader"):
            spec = pair.get(role)
            if spec is not None:
                if not isinstance(spec, dict):
                    raise ValueError(
                        "robot.pairs.{}.{} must be a mapping".format(side, role)
                    )
                yield side, role, spec


def resolve_robot_config(config, interfaces=None):
    """Copy a robot config and resolve selected devices to unique CAN channels.

    Keep the stable selector alongside its resolved channel. On subsequent calls
    it takes precedence over that cached channel, so reconnecting after USB
    re-enumeration never silently falls back to the previous canN name.
    """
    resolved = copy.deepcopy(config)
    used = {}
    for side, role, spec in configured_devices(resolved):
        label = "{}/{}".format(side, role)
        selectors = [key for key in ("usb_serial", "usb_path") if spec.get(key)]
        if len(selectors) > 1:
            raise ValueError(
                "{} must select either usb_serial or usb_path".format(label)
            )
        if selectors:
            selector = selectors[0]
            value = spec[selector]
            if not isinstance(value, str) or not value.strip():
                raise ValueError(
                    "{} {} must be a non-empty string".format(label, selector)
                )
            if interfaces is None:
                interfaces = discover_interfaces()
            matches = [
                item
                for item in interfaces
                if value == item.get(selector)
                or (selector == "usb_path" and value == item.get("usb_interface"))
            ]
            if not matches:
                raise ValueError(
                    "No SocketCAN interface for {} ({}={!r})".format(
                        label, selector, value
                    )
                )
            if len(matches) != 1:
                channels = ", ".join(item["channel"] for item in matches)
                raise ValueError(
                    "Ambiguous SocketCAN interface for {} ({}={!r}): {}. "
                    "Use a unique USB serial or USB interface path.".format(
                        label, selector, value, channels
                    )
                )
            spec["channel"] = matches[0]["channel"]
        channel = spec.get("channel")
        if not isinstance(channel, str) or not _CHANNEL.fullmatch(channel):
            raise ValueError(
                "Invalid SocketCAN channel for {}: {!r}".format(label, channel)
            )
        if channel in used:
            raise ValueError(
                "{} and {} resolve to the same SocketCAN channel {}".format(
                    used[channel], label, channel
                )
            )
        used[channel] = label
    return resolved
