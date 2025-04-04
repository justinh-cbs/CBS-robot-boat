"""
App used to send RTK data to the GPS module

Skeleton GNSS application which continuously receives and parses NMEA, UBX or RTCM
data from a receiver until the stop Event is set or stop() method invoked. Assumes
receiver is connected via serial USB or UART1 port.

"""

# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from pyubx2 import (
    CARRSOLN,
    FIXTYPE,
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
)
from serial import Serial

DISCONNECTED = 0
CONNECTED = 1


class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.recvqueue = kwargs.get("recvqueue", None)
        self.sendqueue = kwargs.get("sendqueue", None)
        self.verbosity = kwargs.get("verbosity", 1)
        self.enableubx = kwargs.get("enableubx", True)
        self.showstatus = kwargs.get("showstatus", False)
        self.stream = None
        self.connected = DISCONNECTED
        self.fix = 0
        self.siv = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0
        self.hacc = 0

    def __enter__(self):
        """
        Context manager enter routine.
        """

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()

    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.connected = CONNECTED
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.recvqueue,
                self.sendqueue,
            ),
            daemon=True,
        )
        read_thread.start()

    def stop(self):
        """
        Stop GNSS reader/writer.
        """

        self.stopevent.set()
        self.connected = DISCONNECTED
        if self.stream is not None:
            self.stream.close()

    def _read_loop(
        self, stream: Serial, stopevent: Event, recvqueue: Queue, sendqueue: Queue
    ):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue recvqueue: queue for messages from receiver
        :param Queue sendqueue: queue for messages to send to receiver
        """

        ubr = UBXReader(
            stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    raw_data, parsed_data = ubr.read()
                    if parsed_data:
                        self._extract_data(parsed_data)
                        if self.verbosity == 1:
                            print(f"GNSS>> {parsed_data.identity}")
                        elif self.verbosity == 2:
                            print(parsed_data)
                        if recvqueue is not None:
                            # place data on receive queue
                            recvqueue.put((raw_data, parsed_data))

                # send any queued output data to receiver
                self._send_data(ubr.datastream, sendqueue)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"Error parsing data stream {err}")
                continue

    def _extract_data(self, parsed_data: object):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
        """

        if hasattr(parsed_data, "fixType"):
            self.fix = FIXTYPE[parsed_data.fixType]
        if hasattr(parsed_data, "carrSoln"):
            self.fix = f"{self.fix} {CARRSOLN[parsed_data.carrSoln]}"
        if hasattr(parsed_data, "numSV"):
            self.siv = parsed_data.numSV
        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm
            self.alt = parsed_data.hMSL / 1000
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 1000
        if hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            self.hacc = parsed_data.hAcc / unit
        if self.showstatus:
            print(
                f"fix {self.fix}, siv {self.siv}, lat {self.lat},",
                f"lon {self.lon}, alt {self.alt:.3f} m, hAcc {self.hacc:.3f} m",
            )

    def _send_data(self, stream: Serial, sendqueue: Queue):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw_data, parsed_data = data
                    if self.verbosity == 1:
                        print(f"GNSS<< {parsed_data.identity}")
                    elif self.verbosity == 2:
                        print(parsed_data)
                    stream.write(raw_data)
                    sendqueue.task_done()
            except Empty:
                pass

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_COR_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (self.connected, self.lat, self.lon, self.alt, self.sep)


if __name__ == "__main__":
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default="/dev/ttyACM1"
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=38400, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
    )
    arp.add_argument(
        "--verbosity",
        required=False,
        help="Verbosity",
        default=1,
        choices=[0, 1, 2],
        type=int,
    )
    arp.add_argument(
        "--enableubx", required=False, help="Enable UBX output", default=1, type=int
    )
    arp.add_argument(
        "--showstatus", required=False, help="Show GNSS status", default=1, type=int
    )

    args = arp.parse_args()
    recv_queue = Queue()  # set to None to print data to stdout
    send_queue = Queue()
    stop_event = Event()

    try:
        print("Starting GNSS reader/writer...\n")
        with GNSSSkeletonApp(
            args.port,
            int(args.baudrate),
            float(args.timeout),
            stop_event,
            recvqueue=recv_queue,
            sendqueue=send_queue,
            verbosity=int(args.verbosity),
            enableubx=int(args.enableubx),
            showstatus=int(args.showstatus),
        ) as gna:
            gna.run()
            while True:
                sleep(1)

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")
