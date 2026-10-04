"""master_link.datalog — binary session log: writer, reader, and format.

Records raw TX/RX frames (and discards/events/timing) as they happen, with a
background writer so nothing blocks the serial hot path. See format.py for the
on-disk layout; writer.py / reader.py for the producer / consumer.
"""
from . import format  # noqa: F401
from .writer import BinaryLogWriter  # noqa: F401
from .reader import BinaryLogReader, read_header, Record  # noqa: F401
