import mmap
import struct
import math
import time
import json
from dataclasses import dataclass


@dataclass
class FieldSpec:
    fmt: str
    name: str
    count: int = 0
    description: str = None

    @property
    def struct_fmt(self):
        if self.count and self.count > 1:
            return f"{self.count}{self.fmt}"
        return self.fmt


def convertDegreeArcToPercent(value):
    return max(value / 360, 0)


FIELDS = [
    # Count, type, name, description
    FieldSpec(fmt="i", name="packetId", description="Current step index"),
    FieldSpec(fmt="f", name="gas", description="Gas pedal input value (from -0 to 1.0)"),
    FieldSpec(fmt="f", name="brake", description="Brake pedal input value (from -0 to 1.0)"),
    FieldSpec(fmt="f", name="fuel", description="Amount of fuel remaining in kg"),
    FieldSpec(fmt="i", name="gear", description="Current gear"),
    FieldSpec(fmt="i", name="rpm", description="Engine revolutions per minute"),
    FieldSpec(fmt="f", name="steerAngle", description="Steering input value (from -1.0 to 1.0)"),
    FieldSpec(fmt="f", name="speedKmh", description="Car speed in km/h"),
    FieldSpec(fmt="f", count=3, name="velocity", description="Car velocity vector in global coordinates"),
    FieldSpec(fmt="f", count=3, name="accG", description="Car acceleration vector in global coordinates"),
    FieldSpec(fmt="f", count=4, name="wheelSlip", description="Tyre slip for each tyre [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="wheelLoad", description="Wheel load for each tyre [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="wheelPressure", description="Tyre pressure [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="wheelAngularSpeed", description="Wheel angular speed in rad/s [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="tyreWear", description="Tyre wear [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="tyreDirtyLevel", description="Dirt accumulated on tyre surface [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="TyreCoreTemp", description="* Tyre rubber core temperature [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="camberRAD", description="Wheels camber in radians [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="suspensionTravel", description="Suspension travel [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", name="drs", description="DRS on"),
    FieldSpec(fmt="f", name="tc", description="** TC in action"),
    FieldSpec(fmt="f", name="heading", description="Car yaw orientation"),
    FieldSpec(fmt="f", name="pitch", description="Car pitch orientation "),
    FieldSpec(fmt="f", name="roll", description="Car roll orientation"),
    FieldSpec(fmt="f", name="cgHeight", description="Centre of gravity height"),
    FieldSpec(fmt="f", count=5, name="carDamage", description="Car damage: front 0, rear 1, left 2, right 3, centre 4"),
    FieldSpec(fmt="i", name="numberOfTyresOut", description="Number of tyres out of track"),
    FieldSpec(fmt="i", name="pitLimiterOn", description="Pit limiter is on"),
    FieldSpec(fmt="f", name="abs", description="ABS in action"),
    FieldSpec(fmt="f", name="kersCharge", description="Not used in ACC"),
    FieldSpec(fmt="f", name="kersInput", description="Not used in ACC"),
    FieldSpec(fmt="i", name="autoshifterOn", description="Automatic transmission on"),
    FieldSpec(fmt="f", count=2, name="rideHeight", description="Ride height: 0 front, 1 rear"),
    FieldSpec(fmt="f", name="turboBoost", description="Car turbo level"),
    FieldSpec(fmt="f", name="ballast", description="Car ballast in kg / Not implemented"),
    FieldSpec(fmt="f", name="airDensity", description="Air density"),
    FieldSpec(fmt="f", name="airTemp", description="Air temperature"),
    FieldSpec(fmt="f", name="roadTemp", description="Road temperature"),
    FieldSpec(fmt="f", count=3, name="localAngularVel", description="Car angular velocity vector in local coordinates"),
    FieldSpec(fmt="f", name="finalFF", description="Force feedback signal"),
    FieldSpec(fmt="f", name="performanceMeter", description="Not used in ACC"),
    FieldSpec(fmt="i", name="engineBrake", description="Not used in ACC"),
    FieldSpec(fmt="i", name="ersRecoveryLevel", description="Not used in ACC"),
    FieldSpec(fmt="i", name="ersPowerLevel", description="Not used in ACC"),
    FieldSpec(fmt="i", name="ersHeatCharging", description="Not used in ACC"),
    FieldSpec(fmt="i", name="ersIsCharging", description="Not used in ACC"),
    FieldSpec(fmt="f", name="kersCurrentKJ", description="Not used in ACC"),
    FieldSpec(fmt="i", name="drsAvailable", description="Not used in ACC"),
    FieldSpec(fmt="i", name="drsEnabled", description="Not used in ACC"),
    FieldSpec(fmt="f", count=4, name="brakeTemp", description="Brake discs temperatures"),
    FieldSpec(fmt="f", name="clutch", description="Clutch pedal input value (from -0 to 1.0)"),
    FieldSpec(fmt="f", count=4, name="tyreTempI", description="Not shown in ACC"),
    FieldSpec(fmt="f", count=4, name="tyreTempM", description="Not shown in ACC"),
    FieldSpec(fmt="f", count=4, name="tyreTempO", description="Not shown in ACC"),
    FieldSpec(fmt="i", name="isAIControlled", description="Car is controlled by the AI"),
    FieldSpec(fmt="f", count=4 * 3, name="tyreContactPoint", description="Tyre contact point global coordinates [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4 * 3, name="tyreContactNormal", description="Tyre contact normal [FL, FR, RL, RR] [x,y,z]"),
    FieldSpec(fmt="f", count=4 * 3, name="tyreContactHeading", description="Tyre contact heading [FL, FR, RL, RR] [x,y,z]"),
    FieldSpec(fmt="f", name="brakeBias", description="Front brake bias, see Appendix 4"),
    FieldSpec(fmt="f", count=3, name="localVelocity", descripton="Car velocity vector in local coordinates"),
    FieldSpec(fmt="i", name="P2PActivation", descripton="Not used in ACC"),
    FieldSpec(fmt="i", name="P2PStatus", descripton="Not used in ACC"),
    FieldSpec(fmt="f", name="currentMaxRpm", descripton="Maximum engine rpm"),
    FieldSpec(fmt="f", count=4, name="mz", descripton="Not shown in ACC"),
    FieldSpec(fmt="f", count=4, name="fx", descripton="Not shown in ACC"),
    FieldSpec(fmt="f", count=4, name="fy", descripton="Not shown in ACC"),
    FieldSpec(fmt="f", count=4, name="slipRatio", descripton="Tyre slip ratio [FL, FR, RL, RR] in radians"),
    FieldSpec(fmt="f", count=4, name="slipAngle", descripton="Tyre slip angle [FL, FR, RL, RR]"),
    FieldSpec(fmt="i", name="tcinAction", descripton="TC in action"),
    FieldSpec(fmt="i", name="absInAction", descripton="ABS in action"),
    FieldSpec(fmt="f", count=4, name="suspensionDamage", descripton="Suspensions damage levels [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="tyreTemp", descripton="Tyres core temperatures [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", name="waterTemp", descripton="Water Temperature"),
    FieldSpec(fmt="f", count=4, name="brakePressure", descripton="Brake pressure [FL, FR, RL, RR] see Appendix 2"),
    FieldSpec(fmt="i", name="frontBrakeCompound", descripton="Brake pad compund front"),
    FieldSpec(fmt="i", name="rearBrakeCompound", descripton="Brake pad compund rear"),
    FieldSpec(fmt="f", count=4, name="padLife", descripton="Brake pad wear [FL, FR, RL, RR]"),
    FieldSpec(fmt="f", count=4, name="discLife", descripton="Brake disk wear [FL, FR, RL, RR]"),
    FieldSpec(fmt="i", name="ignitionOn", descripton="Ignition switch set to on?"),
    FieldSpec(fmt="i", name="starterEngineOn", descripton="Starter Switch set to on?"),
    FieldSpec(fmt="i", name="isEngineRunning", descripton="Engine running?"),
    FieldSpec(fmt="f", name="kerbVibration", descripton="vibrations sent to the FFB, could be used for motion rigs"),
    FieldSpec(fmt="f", name="slipVibrations", descripton="vibrations sent to the FFB, could be used for motion rigs"),
    FieldSpec(fmt="f", name="gVibrations", descripton="vibrations sent to the FFB, could be used for motion rigs"),
    FieldSpec(fmt="f", name="absVibrations", descripton="vibrations sent to the FFB, could be used for motion rigs"),
]


class AssettoCorsaData:
        def __init__(self):
            print('AssettoCorsaData() init()')
            self.layout = self.get_struct_format()
            self.physics_shm_size = struct.calcsize(self.layout)
            self.mmapPhysic = None
            self.mmapStatic = None

        def decode_data(self, raw_values):
            raw_values_iter = iter(raw_values)
            for field in FIELDS:
                read_size = max(1, field.count)
                values = [next(raw_values_iter) for _ in range(read_size)]
                value = values[0] if not field.count else values
                yield field.name, value

        def get_struct_format(self):
            return "".join(x.struct_fmt for x in FIELDS)

        def start(self):
            print('AssettoCorsaData() start()')
            if not self.mmapPhysic:
                self.mmapPhysic = mmap.mmap(-1, self.physics_shm_size, "Local\\acpmf_physics",  access=mmap.ACCESS_READ)
            #self.mmapStatic = mmap.mmap(-1, XYZ, u"Local\\acpmf_static")

        def getData(self):
            self.mmapPhysic.seek(0)
            rawData = self.mmapPhysic.read(self.physics_shm_size)
            raw_values = struct.unpack(self.layout, rawData)

            data = dict(self.decode_data(raw_values))
            # for index, value in enumerate(struct.unpack(self.layout, rawData)):
            #     data[self.fields[index]] = value

            # TODO: make sure whe do this for those fields
            # self._convertData(data)
            return data

        def getJsonData(self):
            return json.dumps(self.getData())

        def stop(self):
            print('AssettoCorsaData() stop()')
            if self.mmapPhysic:
                self.mmapPhysic.close()
            if self.mmapStatic:
                self.mmapStatic.close()

            self.mmapPhysic = None
            self.mmapStatic = None

        def _convertData(self, data):
            # TODO make these conversions immediately when reading from shm
            for newName in ['wheelSlip', 'wheelLoad', 'wheelsPressure',
                    'brakeTemp', 'brakePressure', 'Tyrewear', 'wheelAngularSpeed',
                    'padLife', 'discLife', 'camberRAD', 'TyreCoreTemp', 'tyreDirtyLevel',
                    'suspensionTravel']:
                data[newName] = []
                for oldName in [newName + 'FL', newName+'FR', newName+'RL', newName+'RR']:
                    data[newName].append(convertDegreeArcToPercent(data[oldName]))
                    del data[oldName]


if __name__ == '__main__':
    assettoReader = AssettoCorsaData()
    assettoReader.start()
    while True:
        print('Assetto data:', assettoReader.getData())
        time.sleep(1)
