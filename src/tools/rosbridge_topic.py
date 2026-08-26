#!/usr/bin/env python3
"""Inspect topics on a remote ROS machine through rosbridge WebSocket.

The client running this file does not need ROS installed.  It only needs:

    python3 -m pip install roslibpy

The remote ROS 1 machine must run rosbridge_server, normally on port 9090:

    roslaunch rosbridge_server rosbridge_websocket.launch

Examples:

    python3 rosbridge_topic.py --host 192.168.123.162 list
    python3 rosbridge_topic.py --host 192.168.123.162 list --types
    python3 rosbridge_topic.py --host 192.168.123.162 type /odom
    python3 rosbridge_topic.py --host 192.168.123.162 echo /odom
    python3 rosbridge_topic.py --host 192.168.123.162 echo /odom -n 1
"""

import argparse
import json
import os
import sys
import threading


DEFAULT_HOST = os.environ.get('ROSBRIDGE_HOST', '127.0.1')
DEFAULT_PORT = int(os.environ.get('ROSBRIDGE_PORT', '9090'))


def positive_float(value):
    """Parse a positive floating-point command-line value."""
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return number


def non_negative_float(value):
    """Parse a non-negative floating-point command-line value."""
    number = float(value)
    if number < 0.0:
        raise argparse.ArgumentTypeError('must be zero or greater')
    return number


def non_negative_int(value):
    """Parse a non-negative integer command-line value."""
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError('must be zero or greater')
    return number


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Inspect topics on a remote ROS 1 machine through '
            'roslibpy + rosbridge (no local ROS installation required).'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Environment defaults:\n'
            '  ROSBRIDGE_HOST   rosbridge host (default: 127.0.0.1)\n'
            '  ROSBRIDGE_PORT   rosbridge port (default: 9090)'
        ),
    )
    parser.add_argument(
        '--host', default=DEFAULT_HOST,
        help=f'rosbridge host or IP (default: {DEFAULT_HOST}).',
    )
    parser.add_argument(
        '--port', type=int, default=DEFAULT_PORT,
        help=f'rosbridge WebSocket port (default: {DEFAULT_PORT}).',
    )
    parser.add_argument(
        '--connect-timeout', type=positive_float, default=10.0,
        help='connection timeout in seconds (default: 10).',
    )

    commands = parser.add_subparsers(dest='command', required=True)

    list_parser = commands.add_parser('list', help='list active topic names')
    list_parser.add_argument(
        '-t', '--types', action='store_true',
        help='also query and print each topic message type.',
    )

    type_parser = commands.add_parser('type', help='print a topic message type')
    type_parser.add_argument('topic', help='topic name, for example /odom')

    echo_parser = commands.add_parser('echo', help='print messages from a topic')
    echo_parser.add_argument('topic', help='topic name, for example /odom')
    echo_parser.add_argument(
        '--type', dest='message_type',
        help='message type override; normally it is detected automatically.',
    )
    echo_parser.add_argument(
        '-n', '--count', type=non_negative_int, default=0,
        help='stop after N messages; 0 means keep running (default: 0).',
    )
    echo_parser.add_argument(
        '--timeout', type=non_negative_float, default=0.0,
        help='stop after this many seconds; 0 means no timeout (default: 0).',
    )
    echo_parser.add_argument(
        '--compact', action='store_true',
        help='print each message as one JSON line.',
    )

    args = parser.parse_args()
    if not args.host.strip():
        parser.error('--host must not be empty')
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    return args


def load_roslibpy():
    """Import roslibpy lazily so that --help works without the dependency."""
    try:
        import roslibpy
    except ImportError:
        print(
            'Missing dependency: install it with '
            '`python3 -m pip install roslibpy`.',
            file=sys.stderr,
        )
        raise SystemExit(2)
    return roslibpy


def list_topics(ros, include_types):
    topics = sorted(ros.get_topics())
    for topic in topics:
        if include_types:
            message_type = ros.get_topic_type(topic) or '<unknown>'
            print(f'{topic} [{message_type}]')
        else:
            print(topic)


def print_topic_type(ros, topic):
    message_type = ros.get_topic_type(topic)
    if not message_type:
        raise RuntimeError(f'cannot determine type for topic {topic!r}')
    print(message_type)


def echo_topic(roslibpy, ros, args):
    message_type = args.message_type or ros.get_topic_type(args.topic)
    if not message_type:
        raise RuntimeError(
            f'cannot determine type for topic {args.topic!r}; '
            'check that it exists or pass --type explicitly'
        )

    finished = threading.Event()
    state_lock = threading.Lock()
    message_count = 0

    def on_message(message):
        nonlocal message_count
        with state_lock:
            if finished.is_set():
                return
            message_count += 1
            current_count = message_count

        if args.compact:
            print(
                json.dumps(message, ensure_ascii=False, separators=(',', ':')),
                flush=True,
            )
        else:
            if current_count > 1:
                print('---')
            print(json.dumps(message, ensure_ascii=False, indent=2), flush=True)

        if args.count and current_count >= args.count:
            finished.set()

    listener = roslibpy.Topic(ros, args.topic, message_type)
    print(
        f'Subscribed to {args.topic} [{message_type}] on '
        f'ws://{args.host}:{args.port}; press Ctrl+C to stop.',
        file=sys.stderr,
        flush=True,
    )
    listener.subscribe(on_message)
    try:
        if args.timeout > 0.0:
            completed_by_count = finished.wait(args.timeout)
            if not completed_by_count:
                print(
                    f'No more messages before {args.timeout:g}s timeout.',
                    file=sys.stderr,
                )
        else:
            finished.wait()
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
    finally:
        listener.unsubscribe()


def main():
    args = parse_args()
    roslibpy = load_roslibpy()
    ros = roslibpy.Ros(host=args.host, port=args.port)

    try:
        ros.run(timeout=args.connect_timeout)
        if not ros.is_connected:
            raise RuntimeError('connection did not become ready')

        if args.command == 'list':
            list_topics(ros, args.types)
        elif args.command == 'type':
            print_topic_type(ros, args.topic)
        elif args.command == 'echo':
            echo_topic(roslibpy, ros, args)
    except KeyboardInterrupt:
        print('\nStopped.', file=sys.stderr)
        return 130
    except Exception as exception:
        print(
            f'Error communicating with ws://{args.host}:{args.port}: {exception}',
            file=sys.stderr,
        )
        return 1
    finally:
        try:
            ros.terminate()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
