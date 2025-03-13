"""
This uses the UBXReader and GNSSNTRIPClient classes to get
RTCM3 or SPARTN RTK data from a designated NTRIP
caster/mountpoint and apply it to an RTK-compatible u-blox
GNSS receiver (e.g. ZED-F9P) connected to a local serial
port (USB or UART1).

"""

# pylint: disable=invalid-name

from sys import argv
from queue import Queue, Empty
from threading import Event
from time import sleep
from pathlib import Path

from pygnssutils import VERBOSITY_LOW, GNSSNTRIPClient
from rtk_pub.gnssapp import GNSSSkeletonApp


CONNECTED = 1

# Requires a text file in this directory with the NTRIP password
pw_path = Path(Path.home(), "ros2_ws/src/rtk_pub/rtk_pub/pw.txt")
with open(pw_path, "r") as f:
    PW_STR = f.readline()


def main(**kwargs):
    """
    Main routine.
    """

    SERIAL_PORT = "/dev/ttyUSB0"
    BAUDRATE = 9600
    TIMEOUT = 10

    # Ideally, mountpoint should be <30 km from location.
    IPPROT = "IPv4"
    NTRIP_SERVER = "rtgpsout.earthscope.org"
    NTRIP_PORT = 2101
    HTTPS = 0  # 0 for HTTP, 1 for HTTPS
    FLOWINFO = 0
    SCOPEID = 0
    MOUNTPOINT = "7ODM_RTCM3"  # "P472_RTCM3"  # leave blank to retrieve sourcetable
    NTRIP_USER = "jhelland"
    NTRIP_PASSWORD = PW_STR
    DATATYPE = "RTCM"  # "RTCM" or "SPARTN"

    GGAMODE = 0  # use fixed reference position (0 = use live position)
    GGAINT = 60  # interval in seconds (-1 = do not send NMEA GGA sentences)
    # Fixed reference coordinates (only used when GGAMODE = 1)
    REFLAT = 51.176534
    REFLON = -2.15453
    REFALT = 40.8542
    REFSEP = 26.1743

    recv_queue = Queue()  # data from receiver placed on this queue
    send_queue = Queue()  # data to receiver placed on this queue
    stop_event = Event()
    verbosity = 2  # 0 - no output, 1 - print identities, 2 - print full message

    try:
        print(f"Starting GNSS reader/writer on {SERIAL_PORT} @ {BAUDRATE}...\n")
        with GNSSSkeletonApp(
            SERIAL_PORT,
            BAUDRATE,
            TIMEOUT,
            stopevent=stop_event,
            recvqueue=recv_queue,
            sendqueue=send_queue,
            verbosity=verbosity,
            enableubx=True,
            showstatus=True,
        ) as gna:
            gna.run()
            sleep(2)  # wait for receiver to output at least 1 navigation solution

            print(
                f"Starting NTRIP client on {NTRIP_SERVER}:{NTRIP_PORT}/{MOUNTPOINT}...\n"
            )

            with GNSSNTRIPClient(gna, verbosity=2, logtofile="log.txt") as gnc:
                streaming = gnc.run(
                    ipprot=IPPROT,
                    server=NTRIP_SERVER,
                    port=NTRIP_PORT,
                    https=HTTPS,
                    flowinfo=FLOWINFO,
                    scopeid=SCOPEID,
                    mountpoint=MOUNTPOINT,
                    ntripuser=NTRIP_USER,
                    ntrippassword=NTRIP_PASSWORD,
                    reflat=REFLAT,
                    reflon=REFLON,
                    refalt=REFALT,
                    refsep=REFSEP,
                    ggamode=GGAMODE,
                    ggainterval=GGAINT,
                    datatype=DATATYPE,
                    output=send_queue,  # send NTRIP data to receiver
                )

                while streaming and not stop_event.is_set():
                    if recv_queue is not None:
                        # consume any received GNSS data from queue
                        try:
                            while not recv_queue.empty():
                                (_, parsed_data) = recv_queue.get(False)
                                if verbosity == 1:
                                    print(f"GNSS: {parsed_data.identity}")
                                elif verbosity == 2:
                                    print(parsed_data)
                                recv_queue.task_done()
                        except Empty:
                            pass
                    sleep(1)
                sleep(1)

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")


if __name__ == "__main__":

    main(**dict(arg.split("=") for arg in argv[1:]))
