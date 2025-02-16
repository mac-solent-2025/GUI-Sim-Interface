from dataclasses import dataclass


@dataclass
class NMEA:
    latitude: float = None
    longitude: float = None
    speed: str = None
    heading: str = None

@dataclass
class MMWPL(NMEA):
    latitude: float = None
    longitude: float = None
    waypoint_name: str = None

@dataclass
class RMC_Message(NMEA):
    latitude: float = None
    longitude: float = None
    speed: str = None
    heading: str = None

@dataclass
class TTM_Message(NMEA):
    pass

@dataclass
class Waypoint:
    latitude: str = None
    longitude: str = None
    name: str = None
