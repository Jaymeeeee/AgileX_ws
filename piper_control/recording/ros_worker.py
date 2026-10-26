"""Observation-only ROS process. This module never imports a robot or policy."""
import signal
import sys
import traceback

from piper_control.ipc import read_message, write_message


def serve(incoming, outgoing, bridge_factory=None):
    # This import is confined to the child; SDK workflows import only client.py.
    if bridge_factory is None:
        from .ros_bridge import ROSBridge
        bridge_factory = ROSBridge
    bridge = None
    expected = 0
    try:
        while True:
            try:
                message = read_message(incoming)
            except EOFError:
                return
            kind, sequence = message.get("kind"), message.get("seq")
            try:
                if sequence != expected:
                    raise ValueError("Collection sequence mismatch")
                expected += 1
                if kind == "start" and bridge is None:
                    bridge = bridge_factory(message["config"])
                    bridge.start()
                    if message["config"].get("record", {}).get("backend", "ros") != "ros_publish":
                        bridge.start_bag(message["directory"])
                    reply = {"kind": "ready", "seq": sequence}
                elif bridge is None or kind == "start":
                    raise ValueError("Collection worker startup protocol violation")
                elif kind == "close":
                    bridge.close()
                    bridge = None
                    write_message(outgoing, {"kind": "closed", "seq": sequence})
                    return
                else:
                    payload = message["payload"]
                    if kind == "state":
                        bridge.publish_state(payload)
                    elif kind == "images":
                        bridge.publish_images(payload)
                    elif kind == "action":
                        bridge.publish_action(payload["action"], payload["source"], payload.get("metadata"))
                    elif kind == "event":
                        bridge.publish_event(payload)
                    else:
                        raise ValueError("Unsupported collection input: " + str(kind))
                    bridge.check_health()
                    reply = {"kind": "ack", "seq": sequence}
                write_message(outgoing, reply)
            except Exception as exc:
                traceback.print_exc(file=sys.stderr)
                write_message(outgoing, {"kind": "error", "seq": sequence,
                                        "error": "{}: {}".format(type(exc).__name__, exc)})
                return
    finally:
        if bridge is not None:
            bridge.close()


def main():
    incoming, outgoing = sys.stdin.buffer, sys.stdout.buffer
    sys.stdout = sys.stderr
    def stop(signum, frame):
        raise SystemExit(128 + signum)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    serve(incoming, outgoing)


if __name__ == "__main__":
    main()
